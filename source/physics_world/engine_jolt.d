/**
 * Бэкенд физики на Jolt (Jolt Physics 5.5, C API `JPH`).
 *
 * Наружу отдаёт тот же `PhysWorld`, что и `physics_world.engine_newton`, в
 * car-координатах. Оси Jolt (вверх вдоль Y) наружу не выходят.
 *
 * Связь с C-миром идёт через `bindbc.joltc`: dagon-обёртки (`dagon.ext.jolt`)
 * в проекте нет, а сырые биндинги покрывают всё нужное. Владение телами и
 * формами ведёт сам мир: у Jolt тело — это `JPH_BodyID` в `BodyInterface`,
 * а не объект с деструктором, так что списки мира заменяют `dlib.Owner`.
 */
module physics_world.engine_jolt;

import std.algorithm : max, min;
import std.exception : enforce;
import std.math : PI;

import dlib.core.memory;
import dlib.math.vector;
import dlib.math.quaternion;
import dlib.math.matrix;

import bindbc.joltc;

import physics_world.contactlog : ContactLog;
import physics_world.engine;

/// Гравитация мира, к которой приводится модуль из `PhysBody.gravity`: тело,
/// которому модуль не задавали, падает как на этой машине.
enum float joltDefaultGravity = 9.81f;

/// Шаг миров пула нельзя гонять параллельно: joltc держит для всех систем
/// один TempAllocator, и общий LIFO-стек ломается в "Freeing in the wrong order".
/// Под тем же замком идёт работа с фигурами: пересборка окна земли на ходу
/// создаёт и убивает heightfield посреди чужого шага, и Jolt от этого
/// рассыпается — падает то в CreateShape, то в освобождении шарнира.
private __gshared Object updateLock_ = new Object();

/// Замок на всё время работы с миром, а не только на шаг. Jolt переносит
/// параллельные миры плохо: сцена одного мира (тела, шарниры, фигуры) портит
/// соседний, и это стоит дороже, чем даёт выигрыш в потоках, — замер показал
/// одинаковое время при одном и двух воркерах.
Object joltLock() @property
{
    return updateLock_;
}

/// Радиус скругления формы Jolt. Меньше нуля Jolt не берёт, а большой съедает
/// габариты тонких балок.
private enum float convexRadius = 0.005f;

/* ------------------------------------------------------------------ *
 * Загрузка библиотеки
 * ------------------------------------------------------------------ */

/// Библиотека Jolt грузится один раз на процесс: bindbc resolve символов
/// динамической библиотеки не потокобезопасен, а заезды идут на пуле
/// воркеров. Гвард — ОТКРЫТЫЙ (не потоково-локальный) мьютекс.
private __gshared bool joltLoaded_;
private __gshared Object joltLoadLock_ = new Object();

private extern(C) JPH_ValidateResult joltOnContactValidate(void* userData,
    const(JPH_Body)* body1, const(JPH_Body)* body2,
    const(JPH_RVec3)* baseOffset, const(JPH_CollideShapeResult)* collisionResult);

private extern(C) void joltOnContactAdded(void* userData, const(JPH_Body)* body1,
    const(JPH_Body)* body2, const(JPH_ContactManifold)* manifold,
    JPH_ContactSettings* settings);

void ensureJoltLoaded()
{
    if (joltLoaded_)
        return;
    synchronized (joltLoadLock_)
    {
        if (!joltLoaded_)
        {
            const sup = loadJolt();
            enforce(sup == JoltSupport.v550,
                "Не загрузилась libjoltc.so. Она копируется из dagon:jolt в "
                ~ "корень сборки; при ручном запуске добавь каталог сборки "
                ~ "в LD_LIBRARY_PATH.");
            enforce(JPH_Init(), "JPH_Init() вернула false");
            // В этом C API таблица виртуальных вызовов слушателя одна на
            // процесс, поэтому ставится один раз; различать миры будет
            // `userData` у каждого слушателя.
            // SetProcs хранит переданный указатель, а не копию: структура
            // должна пережить инициализацию.
            static JPH_ContactListener_Procs procs;
            procs.OnContactValidate = &joltOnContactValidate;
            procs.OnContactAdded = &joltOnContactAdded;
            procs.OnContactPersisted = null;
            procs.OnContactRemoved = null;
            JPH_ContactListener_SetProcs(&procs);
            joltLoaded_ = true;
        }
    }
}

/* ------------------------------------------------------------------ *
 * Перевод осей
 *
 * (x, y, z)каркас → (x, z, −y)jolt: мир Jolt — это геометрия каркаса,
 * повёрнутая вокруг X на −90°, ровно как у Newton. Отличие в одном: у Newton
 * «фантомное» тело мастера несёт в локальных осях постоянный разворот, и
 * `worldRotation` приходится править на `worldFrameRotation`. Здесь перевод —
 * честная смена базиса, поэтому обе ручки отдают один и тот же поворот.
 * ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ *
 * Базис
 *
 * Оси Jolt в именах каркаса (frame.frame): right → +x, up → +y, forward → +z.
 * Реализация перевода — в методах `JoltPhysWorld` (контракт `PhysWorld`);
 * геометрия форм живёт в осях тела, которая и есть каркас, — там правок нет.
 * ------------------------------------------------------------------ */

/// Выведено из образов осей базиса, а не через `fromMatrix`: LDC сворачивает
/// `fromMatrix` в NaN на этапе компиляции.
immutable Quaternionf carToJoltQuat =
    Quaternionf(-0.70710678f, 0.0f, 0.0f, 0.70710678f);

private JPH_Vec3 joltGravity(float accel)
{
    return JPH_Vec3(0.0f, -accel, 0.0f);
}

/* ------------------------------------------------------------------ *
 * Слои
 *
 * Роль тела lowered в слой: Jolt решает столкновения по парам слоёв, а роли
 * машины на них не похожи. Балка — сенсор: контакт виден всегда, импульс не
 * передаётся никогда, ровно как `sensor`-тело Newton.
 * ------------------------------------------------------------------ */

private enum JPH_ObjectLayer : uint
{
    /// Земля и препятствия.
    nonMoving = 0,
    /// Рама, рулевая балка, колесо.
    moving = 1,
    /// Балка каркаса: контакт виден, но никогда не толкает.
    sensor = 2,
    count = 3,
}

private enum JPH_BroadPhaseLayer : uint
{
    nonMoving = 0,
    moving = 1,
    count = 2,
}

/// Тело вне коллизии. Jolt не различает «столкновений нет» и «тела нет», но
/// группа с нулевым GroupID не collidится ни с кем.
private JPH_CollisionGroup groupOf(const bool collidable, const ushort subGroup)
{
    JPH_CollisionGroup g;
    g.groupFilter = null;
    g.groupID = collidable ? 1 : 0;
    g.subGroupID = collidable ? subGroup : 0;
    return g;
}

private JPH_ObjectLayer layerOf(const BodyRole role)
{
    final switch (role)
    {
        case BodyRole.ground:
        case BodyRole.obstacle: return JPH_ObjectLayer.nonMoving;
        case BodyRole.master:
        case BodyRole.wheel: return JPH_ObjectLayer.moving;
        case BodyRole.beam: return JPH_ObjectLayer.sensor;
    }
}

/* ------------------------------------------------------------------ *
 * Правила контактов
 *
 * Ключ — пара ролей, а не пара слоёв: правило ставит модель через
 * `setInteraction`, и бэкенд превращает его в поведение Jolt (отказ контакта
 * в валидаторе либо подстановка сцепления и упругости в контакт-листендер).
 * ------------------------------------------------------------------ */

private struct RoleRule
{
    ContactResponse response = ContactResponse.resolve;
    float friction = 0.5f;
    float kineticFriction = 0.5f;
    float elasticity = 0.0f;
}

/// Правила лежат рядом с журналом, потому что контакт-листендер получает
/// единственный `userData` — указатель на контекст мира.
private struct ContactCtx
{
    /// Роли — ordinal edenum, индексируем таблицу ими самими; не заданное
    /// правило — `RoleRule.init`.
    enum brc = BodyRole.max + 1;
    ContactLog log;
    RoleRule[brc][brc] rules;

    private static RoleRule ruleFor(const RoleRule[brc][brc] table,
        const BodyRole a, const BodyRole b)
    {
        const lo = cast(size_t) (a < b ? a : b);
        const hi = cast(size_t) (a < b ? b : a);
        return table[lo][hi];
    }

    private bool ignored(const BodyRole a, const BodyRole b)
    {
        return ruleFor(rules, a, b).response == ContactResponse.ignore;
    }

    /// Модели нужны и сенсорные касания балки, и пары под правилом
    /// `resolveAndReport`; остальное — частный разбор движка.
    private void report(const PhysBody a, const PhysBody b)
    {
        if (a.role == BodyRole.beam || b.role == BodyRole.beam
            || ruleFor(rules, a.role, b.role).response
                == ContactResponse.resolveAndReport)
            log.add(cast(PhysBody) a, cast(PhysBody) b);
    }
}

/// Обёртка тела лежит в памяти мира, а не в GC: контакт-листендер достаёт её
/// по `userData` тела Jolt.
///
/// `userData` хранит указатель на класс, а не на интерфейс, поэтому битый
/// `void*` в `PhysBody` уводил бы вызов в vtable `Object` вместо `PhysBody`.
private PhysBody physOf(const JPH_Body* body)
{
    const ulong raw = JPH_Body_GetUserData(cast(JPH_Body*) body);
    return cast(PhysBody)(cast(JoltPhysBody) cast(void*) cast(size_t) raw);
}

/// Обёртка колбэка: D-исключение не должно уходить в Jolt — там его разматывание
/// оставит все мьютексы тел захваченными, и следующий `DestroyBody` упадёт с
/// EDEADLK вместо нормального отчёта об ошибке.
private extern(C) JPH_ValidateResult joltOnContactValidate(void* userData,
    const(JPH_Body)* body1, const(JPH_Body)* body2,
    const(JPH_RVec3)* baseOffset, const(JPH_CollideShapeResult)* collisionResult)
{
    try
    {
        auto ctx = cast(ContactCtx*) userData;
        if (ctx is null)
            return JPH_ValidateResult.AcceptAllContactsForThisBodyPair;
        auto a = physOf(body1);
        auto b = physOf(body2);
        if (a is null || b is null)
            return JPH_ValidateResult.AcceptAllContactsForThisBodyPair;
        if (ctx.ignored(a.role, b.role))
            return JPH_ValidateResult.RejectAllContactsForThisBodyPair;
        return JPH_ValidateResult.AcceptAllContactsForThisBodyPair;
    }
    catch (Throwable)
    {
        return JPH_ValidateResult.AcceptAllContactsForThisBodyPair;
    }
}

private extern(C) void joltOnContactAdded(void* userData, const(JPH_Body)* body1,
    const(JPH_Body)* body2, const(JPH_ContactManifold)* manifold,
    JPH_ContactSettings* settings)
{
    try
    {
        auto ctx = cast(ContactCtx*) userData;
        if (ctx is null)
            return;
        auto a = physOf(body1);
        auto b = physOf(body2);
        if (a is null || b is null)
            return;
        const RoleRule rule = ctx.ruleFor(ctx.rules, a.role, b.role);
        settings.combinedFriction = rule.friction;
        // TODO: кинетическое трение. Jolt сводит пару к одному коэффициенту
        // сцепления, порога срыва отдельно не знает, поэтому разгон будет
        // отличаться от Newton.
        settings.combinedRestitution = rule.elasticity;
        ctx.report(a, b);
    }
    catch (Throwable)
    {
    }
}

/* ------------------------------------------------------------------ *
 * Формы
 *
 * Форма — непрозрачная обёртка над `JPH_Shape*`: наружу она ничего не
 * говорит, внутрь держит форму Jolt, которую надо отпустить через
 * `JPH_Shape_Destroy`.
 * ------------------------------------------------------------------ */

private final class Shape : PhysShape
{
    JPH_Shape* inner;

    this(JPH_Shape* inner)
    {
        this.inner = inner;
    }
}

/// Сторона сетки Jolt обязана быть степенью двойки: блок высот адресуется
/// сдвигом, а не делением.
private uint pow2Ceil(const uint n)
{
    uint p = 1;
    while (p < n)
        p <<= 1;
    return p;
}

/// Поза тела в car-координатах: кэш, из которого читают геттеры и который
/// переживает пересоздание тела.
struct BodyPose
{
    vec3 pos;
    Quaternionf rot;
}

/* ------------------------------------------------------------------ *
 * Тело
 * ------------------------------------------------------------------ */

final class JoltPhysBody : PhysBody
{
    JoltPhysWorld world_;
    /// Обёртка в сырой памяти мира: мир держит её в `bodies_`, GC её не видит.
    JPH_Body* body_;
    JPH_BodyID id_;

    private BodyRole role_;
    private BodyMotion motion_;
    private size_t tag_;

    private bool collidable_ = true;
    private float mass_ = 1.0f;
    private float gravity_ = joltDefaultGravity;
    private vec3 inertia_ = vec3(0.01f, 0.01f, 0.01f);
    // Без явных нулей D даёт float в NaN: смещение ЦМ уехало бы в форму, а
    // демпфирование — в интегратор.
    private vec3 comOffset_ = vec3(0.0f, 0.0f, 0.0f);
    private float linearDamping_ = 0.0f;
    private vec3 angularDamping_ = vec3(0.0f, 0.0f, 0.0f);

    /// Массовые свойства Jolt задаёт только при создании тела, поэтому их
    /// изменение — пересоздание. Модель собирает шарниры после масс, и до
    /// этого момента пересоздание безопасно.
    private bool jointed_;

    this(JoltPhysWorld w, BodyRole role, BodyMotion motion, Shape shape,
        float mass)
    {
        world_ = w;
        role_ = role;
        motion_ = motion;
        mass_ = mass > 0.0f ? mass : 1.0f;
        inertia_ = vec3(mass_ * 0.01f, mass_ * 0.01f, mass_ * 0.01f);
        createBody(shape);
    }

    private void createBody(Shape shape)
    {
        JPH_MotionType mt;
        final switch (motion_)
        {
            case BodyMotion.staticBody: mt = JPH_MotionType.Static; break;
            case BodyMotion.kinematicBody: mt = JPH_MotionType.Kinematic; break;
            case BodyMotion.dynamicBody: mt = JPH_MotionType.Dynamic; break;
        }
        // Форму с офсетом ЦМ оборачиваем: смещение задаётся в локальных осях
        // тела, переводить их в оси Jolt нельзя — иначе ось колеса съедет на
        // борт.
        JPH_Shape* geom = shape.inner;
        if (comOffset_ != vec3(0.0f, 0.0f, 0.0f))
            synchronized (updateLock_)
            {
                JPH_Vec3 off = world_.toEngineDir(comOffset_);
                JPH_OffsetCenterOfMassShapeSettings* cs =
                    JPH_OffsetCenterOfMassShapeSettings_Create2(&off, geom);
                geom = cast(JPH_Shape*)
                    JPH_OffsetCenterOfMassShapeSettings_CreateShape(cs);
                JPH_ShapeSettings_Destroy(cast(JPH_ShapeSettings*) cs);
            }

        const auto pose = world_.pose(id_);
        JPH_RVec3 pos = world_.toEnginePos(pose.pos);
        JPH_Quat rot = world_.toEngineRot(pose.rot);
        JPH_BodyCreationSettings* cs = JPH_BodyCreationSettings_Create3(geom,
            &pos, &rot, mt, layerOf(role_));
        JPH_BodyCreationSettings_SetUserData(cs,
            cast(ulong) cast(size_t) cast(void*) this);
        JPH_BodyCreationSettings_SetAllowSleeping(cs, false);
        // Сцепление и упругость приходят из таблицы ролей в контакт-листендере,
        // поэтому у тела они нулевые иначе пара перемножилась бы с лишним.
        JPH_BodyCreationSettings_SetFriction(cs, 0.0f);
        JPH_BodyCreationSettings_SetRestitution(cs, 0.0f);
        JPH_BodyCreationSettings_SetLinearDamping(cs, linearDamping_);
        JPH_BodyCreationSettings_SetAngularDamping(cs, angularDamping_.x);
        JPH_BodyCreationSettings_SetGravityFactor(cs, gravity_ / joltDefaultGravity);
        JPH_CollisionGroup grp = groupOf(collidable_, 0);
        JPH_BodyCreationSettings_SetCollisionGroup(cs, &grp);
        if (mt == JPH_MotionType.Dynamic)
        {
            JPH_MassProperties mp;
            mp.mass = mass_;
            mp.inertia = inertiaMatrix(inertia_);
            JPH_BodyCreationSettings_SetMassPropertiesOverride(cs, &mp);
            JPH_BodyCreationSettings_SetOverrideMassProperties(cs,
                JPH_OverrideMassProperties.MassAndInertiaProvided);
        }
        if (role_ == BodyRole.beam)
            JPH_BodyCreationSettings_SetIsSensor(cs, true);

        body_ = JPH_BodyInterface_CreateBody(world_.bodyInterface_, cs);
        JPH_BodyCreationSettings_Destroy(cs);
        enforce(body_ !is null, "Jolt не создал тело");
        id_ = JPH_Body_GetID(body_);
        JPH_BodyInterface_AddBody(world_.bodyInterface_, id_,
            JPH_Activation.Activate);
        world_.track(this);
        syncPose();
    }

    /// Jolt ждёт момент инерции матрицей в локальных осях тела: главные
    /// моменты модели кладутся на диагональ.
    private static Matrix4x4f inertiaMatrix(const vec3 principal)
    {
        Matrix4x4f m = Matrix4x4f.identity;
        m[0, 0] = principal.x;
        m[1, 1] = principal.y;
        m[2, 2] = principal.z;
        return m;
    }

    private ref pose() @property
    {
        return world_.pose(id_);
    }

    private void recreate()
    {
        enforce(!jointed_,
            "массовые свойства тела меняются только до шарниров");
        auto shape = cast(Shape) shape();
        world_.untrack(this);
        destroy();
        createBody(shape);
    }

    private void destroy()
    {
        if (body_ is null)
            return;
        JPH_BodyInterface_RemoveAndDestroyBody(world_.bodyInterface_, id_);
        body_ = null;
    }

    private JPH_MotionType motionType() const
    {
        return JPH_Body_GetMotionType(cast(JPH_Body*) body_);
    }

    private JPH_MotionProperties* motionProps() const
    {
        return JPH_Body_GetMotionPropertiesUnchecked(cast(JPH_Body*) body_);
    }

    override PhysWorld physWorld() @property { return world_; }
    override BodyRole role() @property const { return role_; }
    override void tag(size_t t) @property { tag_ = t; }
    override size_t tag() @property const { return tag_; }
    override PhysShape shape() @property
    {
        return New!Shape(cast(JPH_Shape*) JPH_Body_GetShape(
            cast(JPH_Body*) body_));
    }

    override Vector3f worldPosition() @property { return pose().pos; }
    override Quaternionf worldRotation() @property { return pose().rot; }

    /// У Jolt перевод осей — смена базиса, а не разворот локальных осей
    /// фантомного тела, поэтому поправки к повороту нет: обе ручки отдают
    /// истинный поворот в осях каркаса.
    override Quaternionf worldFrameRotation() @property { return pose().rot; }

    override void worldRotation(Quaternionf carRot) @property
    {
        // Вокруг текущего центра масс: иначе тело уедет из-под привязанных
        // к нему колёс.
        const vec3 com = worldCenterOfMass;
        pose().pos = com + carRot.rotate(pose().pos - com);
        pose().rot = carRot;
        pushPose();
    }

    override void worldTransform(const vec3 carPos, const Quaternionf carRot) @property
    {
        pose().pos = carPos;
        pose().rot = carRot;
        pushPose();
    }

    override void setWorldPosition(const vec3 carPos)
    {
        pose().pos = carPos;
        pushPose();
    }

    override Vector3f velocity() @property
    {
        JPH_Vec3 v;
        JPH_BodyInterface_GetLinearVelocity(world_.bodyInterface_, id_, &v);
        return world_.toCarDir(v);
    }

    override void velocity(Vector3f v) @property
    {
        JPH_Vec3 jv = world_.toEngineDir(v);
        JPH_BodyInterface_SetLinearVelocity(world_.bodyInterface_, id_, &jv);
    }

    override Vector3f angularVelocity() @property
    {
        JPH_Vec3 w;
        JPH_BodyInterface_GetAngularVelocity(world_.bodyInterface_, id_, &w);
        return world_.toCarDir(w);
    }

    override void angularVelocity(Vector3f w) @property
    {
        JPH_Vec3 jw = world_.toEngineDir(w);
        JPH_BodyInterface_SetAngularVelocity(world_.bodyInterface_, id_, &jw);
    }

    override Vector3f worldCenterOfMass() @property
    {
        JPH_RVec3 com;
        JPH_BodyInterface_GetCenterOfMassPosition(world_.bodyInterface_, id_, &com);
        return world_.toCarPos(com);
    }

    override void localCenterOfMass(Vector3f offset) @property
    {
        comOffset_ = offset;
        recreate();
    }

    override void mass(float m) @property
    {
        mass_ = m > 0.0f ? m : 1.0f;
        recreate();
    }

    override void inertia(Vector3f principal) @property
    {
        inertia_ = principal;
        recreate();
    }

    override void gravity(float accel) @property
    {
        gravity_ = accel;
        JPH_BodyInterface_SetGravityFactor(world_.bodyInterface_, id_,
            accel / joltDefaultGravity);
    }

    override void linearDamping(float damping) @property
    {
        linearDamping_ = damping;
        JPH_MotionProperties* mp = motionProps();
        if (mp !is null)
            JPH_MotionProperties_SetLinearDamping(mp, damping);
    }

    override void angularDamping(Vector3f damping) @property
    {
        // Jolt не различает демпфирование по осям, поэтому берётся максимум
        // из трёх: иначе рама потеряет энергию быстрее, чем на Newton.
        // TODO: покомпонентное демпфирование.
        angularDamping_ = damping;
        JPH_MotionProperties* mp = motionProps();
        if (mp !is null)
            JPH_MotionProperties_SetAngularDamping(mp,
                max(damping.x, max(damping.y, damping.z)));
    }

    override void collidable(bool on) @property
    {
        collidable_ = on;
        JPH_CollisionGroup g = groupOf(on, 0);
        JPH_BodyInterface_SetCollisionGroup(world_.bodyInterface_, id_, &g);
    }

    override void addForce(Vector3f f)
    {
        JPH_Vec3 jf = world_.toEngineDir(f);
        JPH_BodyInterface_AddForce(world_.bodyInterface_, id_, &jf);
    }

    override void addTorque(Vector3f t)
    {
        JPH_Vec3 jt = world_.toEngineDir(t);
        JPH_BodyInterface_AddTorque(world_.bodyInterface_, id_, &jt);
    }

    /// Сеттеры позы переносят тело сразу: балки — сенсоры, им `MoveKinematic`
    /// не нужен, а фиксация позы каждый шаг совпадает с поведением Newton.
    private void pushPose()
    {
        if (body_ is null)
            return;
        JPH_RVec3 pos = world_.toEnginePos(pose().pos);
        JPH_Quat rot = world_.toEngineRot(pose().rot);
        JPH_BodyInterface_SetPositionAndRotation(world_.bodyInterface_, id_,
            &pos, &rot,
            motionType() == JPH_MotionType.Static
                ? JPH_Activation.DontActivate : JPH_Activation.Activate);
    }

    /// Как у Newton: обёртка читает из движка свежую позу.
    override void syncPose()
    {
        readBackPose();
    }

    /// Забрать позу из Jolt в кэш: после шага геттеры обязаны видеть результат
    /// без дополнительного запроса в движок.
    void readBackPose()
    {
        if (body_ is null)
            return;
        JPH_RVec3 pos;
        JPH_Quat rot;
        JPH_BodyInterface_GetPositionAndRotation(world_.bodyInterface_, id_, &pos,
            &rot);
        pose().pos = world_.toCarPos(pos);
        pose().rot = world_.toCarRot(rot);
    }

    void dispose()
    {
        destroy();
        world_.untrack(this);
    }
}

/* ------------------------------------------------------------------ *
 * Шарниры
 * ------------------------------------------------------------------ */

/// Ось колеса: ступица держится на раме и свободно вращается вокруг своей оси.
/// Hinge Jolt держит точку стыка и ось — ровно то, что дают три линейных и
/// два угловых ряда шарнира Newton, но решается это штатно, а не
/// пользовательским ограничением.
private final class AxleJoint : PhysAxleJoint
{
    private PhysBody physA_, physB_;
    private JPH_HingeConstraint* constraint_;

    this(PhysBody wheel, PhysBody master, const vec3 pivotMaster)
    {
        auto w = cast(JoltPhysBody) wheel;
        auto m = cast(JoltPhysBody) master;
        physA_ = w;
        physB_ = m;
        // Ось колеса — локальная Y колеса; в осях рамы её даёт поза ступицы на
        // момент сборки, а не кэш рамы: она могла уже быть повёрнута.
        const vec3 axleCar = w.pose().rot.rotate(Vector3f(0.0f, 1.0f, 0.0f));
        const vec3 pointCar = m.pose().pos + m.pose().rot.rotate(pivotMaster);
        JPH_HingeConstraintSettings s;
        // D инициализирует float в NaN, а Jolt строит по normalAxis базис угла
        // шарнира: без штатных значений решатель выдаёт NaN-скорости тел.
        JPH_HingeConstraintSettings_Init(&s);
        s.base.enabled = true;
        s.space = JPH_ConstraintSpace.WorldSpace;
        s.point1 = m.world_.toEnginePos(pointCar);
        s.hingeAxis1 = m.world_.toEngineDir(axleCar);
        s.point2 = s.point1;
        s.hingeAxis2 = s.hingeAxis1;
        // Базис угла строится из нормали, и она обязана быть перпендикулярна
        // оси шарнира, иначе якобиан вырожден и решатель расходится.
        vec3 sideCar = axleCar.cross(vec3(0.0f, 0.0f, 1.0f));
        if (sideCar.lengthsqr < 1e-6f)
            sideCar = axleCar.cross(vec3(1.0f, 0.0f, 0.0f));
        const JPH_Vec3 side = m.world_.toEngineDir(sideCar.normalized());
        s.normalAxis1 = side;
        s.normalAxis2 = side;
        s.limitsMin = -PI;
        s.limitsMax = PI;
        s.maxFrictionTorque = 0.0f;
        constraint_ = JPH_HingeConstraint_Create(&s, m.body_, w.body_);
        m.world_.addConstraint(cast(JPH_Constraint*) constraint_);
        m.jointed_ = true;
        w.jointed_ = true;
    }

    override PhysBody bodyA() @property { return physA_; }
    override PhysBody bodyB() @property { return physB_; }

    void dispose()
    {
        if (constraint_ is null)
            return;
        auto w = (cast(JoltPhysBody) physB_).world_;
        w.destroyConstraint(cast(JPH_Constraint*) constraint_);
        constraint_ = null;
    }
}

/// TODO: рулевой шарнир. У Jolt это `HingeConstraint` с приводом и пределом
/// хода между балкой и рамой, но модель требует ряд с потолком ошибки
/// подпитки (без него ненагруженная балка раскручивает сама себя), а такого
/// ряда в Jolt нет. Пока заглушка: руль не удерживает курс, `yawNow` честно
/// отдаёт ноль, и тесты руля для этого бэкенда отключены.
private final class SteerJoint : PhysSteerJoint
{
    private PhysBody physA_, physB_;
    private float targetYaw_;

    this(PhysBody steer, PhysBody master, const vec3 pivotSteer,
        const vec3 pivotMaster, const vec3 armUp, float limit, float errorCap)
    {
        physA_ = steer;
        physB_ = master;
    }

    override PhysBody bodyA() @property { return physA_; }
    override PhysBody bodyB() @property { return physB_; }
    override void targetYaw(float yaw) @property { targetYaw_ = yaw; }
    override float targetYaw() @property const { return targetYaw_; }
    override float yawNow() @property const { return 0.0f; }
}

/* ------------------------------------------------------------------ *
 * Мир
 * ------------------------------------------------------------------ */

final class JoltPhysWorld : PhysWorld
{
    private JPH_PhysicsSystem* system_;
    /// Интерфейс тел: наружу не выходит, тела ходят по нему отсюда.
    JPH_BodyInterface* bodyInterface_;
    private JPH_JobSystem* jobs_;

    private JPH_ContactListener* listener_;
    private JPH_BroadPhaseLayerInterface* bpLayers_;
    private JPH_ObjectLayerPairFilter* objFilter_;
    private JPH_ObjectVsBroadPhaseLayerFilter* objVsBpFilter_;

    /// Всё, что мир отдал за заезд: формы, тела и шарниры живут до конца
    /// заезда, а пул миры не уничтожает — без этих списков мир копил бы их
    /// до своей гибели.
    private void*[] shapes_;
    private JPH_Body[] bodies_;
    private JPH_BodyID[] bodyIds_;
    private JoltPhysBody[] bodiesOwned_;
    private AxleJoint[] joints_;
    private void*[] constraints_;
    private BodyPose[JPH_BodyID] poses_;

    /// Пул потоков Jolt: свой на мир, по одному потоку. Ноль означает, что
    /// работу делает вызывающий поток — этого просит `useCallingThread`.
    private int jobThreads_ = 1;

    ContactCtx ctx_;

    this()
    {
        ensureJoltLoaded();
        createSystem();
    }

    override immutable vec3 right() @property
    {
        return vec3(1.0f, 0.0f, 0.0f);
    }

    override immutable vec3 forward() @property
    {
        return vec3(0.0f, -1.0f, 0.0f);
    }

    override immutable vec3 up() @property
    {
        return vec3(0.0f, 0.0f, 1.0f);
    }

    override vec3 toEnginePos(const vec3 carPos)
    {
        return vec3(carPos.x, carPos.z, -carPos.y);
    }

    override vec3 toCarPos(const vec3 joltPos)
    {
        return vec3(joltPos.x, -joltPos.z, joltPos.y);
    }

    override Quaternionf toEngineRot(const Quaternionf carRot)
    {
        Quaternionf c = carToJoltQuat;
        return c * carRot;
    }

    override Quaternionf toCarRot(const Quaternionf joltRot)
    {
        Quaternionf c = carToJoltQuat;
        return c.conj * joltRot;
    }

    /// Вектор поворачивается тем же поворотом, что и тело: ось угловой
    /// скорости живёт в мире, а не в системе тела.
    override vec3 toEngineDir(const vec3 carDir)
    {
        return vec3(carDir.x, carDir.z, -carDir.y);
    }

    override vec3 toCarDir(const vec3 joltDir)
    {
        Quaternionf c = carToJoltQuat;
        return c.conj.rotate(joltDir);
    }

    private void createSystem()
    {
        bpLayers_ = JPH_BroadPhaseLayerInterfaceTable_Create(
            JPH_ObjectLayer.count, JPH_BroadPhaseLayer.count);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpLayers_,
            JPH_ObjectLayer.nonMoving, JPH_BroadPhaseLayer.nonMoving);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpLayers_,
            JPH_ObjectLayer.moving, JPH_BroadPhaseLayer.moving);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpLayers_,
            JPH_ObjectLayer.sensor, JPH_BroadPhaseLayer.moving);

        objFilter_ = JPH_ObjectLayerPairFilterTable_Create(JPH_ObjectLayer.count);
        // В Jolt таблица пар по умолчанию пустая: без включений не сталкивается
        // ничего. Землю с землёй и балку с балкой оставляем выключенными.
        JPH_ObjectLayerPairFilterTable_EnableCollision(objFilter_,
            JPH_ObjectLayer.moving, JPH_ObjectLayer.nonMoving);
        JPH_ObjectLayerPairFilterTable_EnableCollision(objFilter_,
            JPH_ObjectLayer.moving, JPH_ObjectLayer.moving);
        JPH_ObjectLayerPairFilterTable_EnableCollision(objFilter_,
            JPH_ObjectLayer.sensor, JPH_ObjectLayer.nonMoving);
        JPH_ObjectLayerPairFilterTable_EnableCollision(objFilter_,
            JPH_ObjectLayer.sensor, JPH_ObjectLayer.moving);

        objVsBpFilter_ = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(bpLayers_,
            JPH_BroadPhaseLayer.count, objFilter_, JPH_ObjectLayer.count);

        JPH_PhysicsSystemSettings ps;
        ps.maxBodies = 2048;
        ps.numBodyMutexes = 0;
        ps.maxBodyPairs = 8192;
        ps.maxContactConstraints = 4096;
        ps.broadPhaseLayerInterface = bpLayers_;
        ps.objectLayerPairFilter = objFilter_;
        ps.objectVsBroadPhaseLayerFilter = objVsBpFilter_;
        system_ = JPH_PhysicsSystem_Create(&ps);
        enforce(system_ !is null, "Jolt не создал систему физики");
        bodyInterface_ = JPH_PhysicsSystem_GetBodyInterface(system_);
        JPH_Vec3 g = joltGravity(joltDefaultGravity);
        JPH_PhysicsSystem_SetGravity(system_, &g);
        // Заезд длится доли секунды: сон тела только рассинхронизирует раму с
        // кинематическими балками, которые продолжают за ней идти.
        JPH_PhysicsSettings settings;
        JPH_PhysicsSystem_GetPhysicsSettings(system_, &settings);
        settings.allowSleeping = false;
        JPH_PhysicsSystem_SetPhysicsSettings(system_, &settings);
        listener_ = JPH_ContactListener_Create(cast(void*) &ctx_);
        JPH_PhysicsSystem_SetContactListener(system_, listener_);
        createJobs();
    }

    private void createJobs()
    {
        if (jobThreads_ > 0)
        {
            JobSystemThreadPoolConfig cfg;
            cfg.maxJobs = 512;
            cfg.maxBarriers = 8;
            cfg.numThreads = jobThreads_;
            jobs_ = JPH_JobSystemThreadPool_Create(&cfg);
        }
        else
        {
            // Джобы C++ гонят вне D-рантайма: воркер не регистрируется в GC и
            // колбэки контактов, читающие ассоциативные массивы мира, ловят
            // сборку памяти под ногами. Вызывающему потоку это не грозит.
            JPH_JobSystemConfig cfg;
            cfg.queueJob = &runInline;
            cfg.queueJobs = &runInlineMany;
            cfg.maxConcurrency = 1;
            cfg.maxBarriers = 8;
            jobs_ = JPH_JobSystemCallback_Create(&cfg);
        }
    }

    private static extern(C) void runInline(void* context, JPH_JobFunction job, void* arg)
    {
        job(arg);
    }

    private static extern(C) void runInlineMany(void* context, JPH_JobFunction job, void** args, uint count)
    {
        foreach (i; 0 .. count)
            job(args[i]);
    }

    override void useCallingThread()
    {
        jobThreads_ = 0;
        if (jobs_ !is null)
        {
            JPH_JobSystem_Destroy(jobs_);
            jobs_ = null;
            createJobs();
        }
    }

    override void step(double dt)
    {
        ctx_.log.clear();
        synchronized (updateLock_)
            JPH_PhysicsSystem_Update(system_, cast(float) dt, 1, jobs_);
        foreach (b; bodiesOwned_)
            b.readBackPose();
    }

    override const(ContactPair)[] contacts() const { return ctx_.log.pairs; }

    override PhysShape boxShape(const vec3 halfExtent)
    {
        JPH_Vec3 h = halfExtent;
        JPH_BoxShapeSettings* cs = JPH_BoxShapeSettings_Create(&h, convexRadius);
        synchronized (updateLock_)
            return keepShape(cast(JPH_Shape*)
                JPH_BoxShapeSettings_CreateShape(cs), cs);
    }

    /// Ось цилиндра Jolt уже вдоль локальной Y — как требует модель, — в
    /// отличие от Newton, где форму приходилось доворачивать. Покрышка колеса
    /// сужается к ободу, поэтому форма конусная.
    override PhysShape cylinderShape(float radius1, float radius2, float height)
    {
        synchronized (updateLock_)
        {
            if (radius1 != radius2)
            {
                auto cs = JPH_TaperedCylinderShapeSettings_Create(height * 0.5f,
                    radius1, radius2, convexRadius, null);
                return keepShape(cast(JPH_Shape*)
                    JPH_TaperedCylinderShapeSettings_CreateShape(cs), cs);
            }
            auto cy = JPH_CylinderShapeSettings_Create(height * 0.5f, radius1,
                convexRadius);
            return keepShape(cast(JPH_Shape*)
                JPH_CylinderShapeSettings_CreateShape(cy), cy);
        }
    }

    /// Статическая геометрия: Jolt строит дерево по индексам, нормали вершин
    /// ему не нужны.
    override PhysShape triangleMeshShape(const float[] vertices,
        const float[] normals, const uint[] indices)
    {
        const uint nv = cast(uint) vertices.length / 3;
        auto vs = new JPH_Vec3[nv];
        foreach (i; 0 .. nv)
            vs[i] = toEnginePos(Vector3f(vertices[i * 3], vertices[i * 3 + 1],
                vertices[i * 3 + 2]));
        const uint nt = cast(uint) indices.length / 3;
        auto ts = new JPH_IndexedTriangle[nt];
        foreach (i; 0 .. nt)
        {
            ts[i].i1 = indices[i * 3];
            ts[i].i2 = indices[i * 3 + 1];
            ts[i].i3 = indices[i * 3 + 2];
            ts[i].materialIndex = 0;
        }
        JPH_MeshShapeSettings* cs = JPH_MeshShapeSettings_Create2(vs.ptr, nv,
            ts.ptr, nt);
        synchronized (updateLock_)
            return keepShape(cast(JPH_Shape*)
                JPH_MeshShapeSettings_CreateShape(cs), cs);
    }

    /// Ровная земля: плоскость Jolt бесконечна и стоит в нуле по высоте, а тело
    /// встаёт на отметку `topZ`.
    override PhysBody createFlatGround(float halfExtent, uint cells, float topZ)
    {
        JPH_Plane plane;
        // Поверхность вверх — car-z, в осях тела после басиса это локальный +z.
        plane.normal = JPH_Vec3(0.0f, 0.0f, 1.0f);
        // Тело уже стоит на topZ: уравнению не нужен второй сдвиг.
        plane.distance = 0.0f;
        JPH_PlaneShapeSettings* cs = JPH_PlaneShapeSettings_Create(&plane, null,
            2.0f * halfExtent);
        PhysShape shape;
        synchronized (updateLock_)
            shape = keepShape(cast(JPH_Shape*)
                JPH_PlaneShapeSettings_CreateShape(cs), cs);
        auto b = createBody(BodyRole.ground, BodyMotion.staticBody, shape, 0.0f);
        b.setWorldPosition(vec3(0.0f, 0.0f, topZ));
        b.syncPose();
        return b;
    }

    /// Окно terrain: сетка высот приходит в car-координатах, а Jolt требует
    /// сторону степенью двойки, поэтому окно дополняется краями по крайним
    /// отметкам, а тело ставится так, чтобы поле накрыло окно целиком.
    override PhysBody createHeightfieldGround(const float[] heights, uint dims,
        float cell, const vec3 corner)
    {
        const uint n = pow2Ceil(dims);
        auto samples = new float[cast(size_t) n * n];
        foreach (j; 0 .. n)
        {
            const uint f = min(j, dims - 1);
            foreach (i; 0 .. n)
            {
                const uint r = min(i, dims - 1);
                samples[cast(size_t) j * n + i] =
                    heights[cast(size_t) f * dims + r];
            }
        }
        JPH_Vec3 offset = JPH_Vec3(0.0f, 0.0f, 0.0f);
        JPH_Vec3 scale = JPH_Vec3(cell, 1.0f, cell);
        JPH_HeightFieldShapeSettings* cs = JPH_HeightFieldShapeSettings_Create(
            samples.ptr, &offset, &scale, n, null);
        PhysShape shape;
        synchronized (updateLock_)
            shape = keepShape(cast(JPH_Shape*)
                JPH_HeightFieldShapeSettings_CreateShape(cs), cs);
        auto b = createBody(BodyRole.ground, BodyMotion.staticBody, shape, 0.0f);
        // Высоты Jolt идут по локальной +Y, а вертикаль рельефа — up (car-z);
        // поворот тела не совместит оси (нужно зеркало), поэтому поле ставится
        // без разворота: локальные x/y/z совпадают с up/forward/right каркаса.
        JPH_RVec3 groundPos = vec3(corner.x, 0.0f, -corner.y);
        JPH_Quat groundRot = Quaternionf.identity;
        auto jb = cast(JoltPhysBody) b;
        JPH_BodyInterface_SetPositionAndRotation(bodyInterface_,
            JPH_Body_GetID(cast(JPH_Body*) jb.body_), &groundPos, &groundRot,
            JPH_Activation.DontActivate);
        return b;
    }

    override PhysBody createBody(BodyRole role, BodyMotion motion,
        PhysShape shape, float mass)
    {
        auto s = cast(Shape) shape;
        enforce(s !is null, "форма выдана не этому бэкенду");
        return New!JoltPhysBody(this, role, motion, s, mass);
    }

    override void destroyBody(PhysBody body)
    {
        auto b = cast(JoltPhysBody) body;
        if (b !is null)
            b.dispose();
    }

    override void destroyShape(PhysShape shape)
    {
        auto s = cast(Shape) shape;
        if (s is null || s.inner is null)
            return;
        JPH_Shape* inner = s.inner;
        s.inner = null;
        untrackShape(cast(void*) inner);
        synchronized (updateLock_)
            JPH_Shape_Destroy(inner);
    }

    override PhysAxleJoint newAxleJoint(PhysBody wheel, PhysBody master,
        const vec3 pivotMaster)
    {
        auto j = New!AxleJoint(wheel, master, pivotMaster);
        joints_ ~= j;
        return j;
    }

    override PhysSteerJoint newSteerJoint(PhysBody steer, PhysBody master,
        const vec3 pivotSteer, const vec3 pivotMaster, const vec3 armUp,
        float limit, float errorCap)
    {
        return New!SteerJoint(steer, master, pivotSteer, pivotMaster, armUp,
            limit, errorCap);
    }

    override void setInteraction(in Interaction rule)
    {
        const BodyRole lo = rule.a < rule.b ? rule.a : rule.b;
        const BodyRole hi = rule.a < rule.b ? rule.b : rule.a;
        RoleRule r;
        r.response = rule.response;
        r.friction = rule.friction;
        r.kineticFriction = rule.kineticFriction;
        r.elasticity = rule.elasticity;
        ctx_.rules[cast(size_t) lo][cast(size_t) hi] = r;
    }

    /// Настройки отпускаются сразу же, форма живёт в списке до конца заезда.
    private PhysShape keepShape(T)(JPH_Shape* inner, T* cs)
    {
        // Jolt на негодные настройки отдаёт пустую форму, а пустая форма
        // валит симуляцию позже и в чужом месте.
        enforce(inner !is null, "Jolt отверг настройки формы");
        JPH_ShapeSettings_Destroy(cast(JPH_ShapeSettings*) cs);
        shapes_ ~= cast(void*) inner;
        return New!Shape(inner);
    }

    private void untrackShape(void* inner)
    {
        foreach (i, sh; shapes_)
            if (sh is inner)
            {
                shapes_[i] = shapes_[$ - 1];
                shapes_.length = shapes_.length - 1;
                return;
            }
    }

    void addConstraint(JPH_Constraint* c)
    {
        constraints_ ~= cast(void*) c;
        JPH_PhysicsSystem_AddConstraint(system_, c);
    }

    void destroyConstraint(JPH_Constraint* c)
    {
        JPH_PhysicsSystem_RemoveConstraint(system_, c);
        foreach (i; 0 .. constraints_.length)
            if (constraints_[i] is cast(void*) c)
            {
                constraints_[i] = constraints_[$ - 1];
                constraints_.length = constraints_.length - 1;
                break;
            }
        JPH_Constraint_Destroy(c);
    }

    /// Поза тела: кэш для геттеров, переживающий пересоздание тела.
    ref pose(JPH_BodyID id)
    {
        auto p = id in poses_;
        if (p is null)
        {
            // .init у float — NaN, а кэш обязан стартовать с единичного
            // поворота: тело создаётся в нуле, а не в NaN.
            BodyPose d;
            d.pos = vec3(0.0f, 0.0f, 0.0f);
            d.rot = Quaternionf.identity;
            poses_[id] = d;
        }
        return poses_[id];
    }

    void track(JoltPhysBody b)
    {
        bodyIds_ ~= b.id_;
        bodiesOwned_ ~= b;
    }

    void untrack(JoltPhysBody b)
    {
        foreach (i; 0 .. bodiesOwned_.length)
            if (bodiesOwned_[i] is b)
            {
                bodyIds_[i] = bodyIds_[$ - 1];
                bodyIds_.length = bodyIds_.length - 1;
                bodiesOwned_[i] = bodiesOwned_[$ - 1];
                bodiesOwned_.length = bodiesOwned_.length - 1;
                break;
            }
        poses_.remove(b.id_);
    }

    override void clearScene()
    {
        // Порядок обязателен: шарнир ссылается на тела, а тело — на форму.
        foreach (j; joints_)
            j.dispose();
        joints_ = null;
        foreach (c; constraints_)
            JPH_Constraint_Destroy(cast(JPH_Constraint*) c);
        constraints_ = null;
        foreach (id; bodyIds_)
            JPH_BodyInterface_RemoveAndDestroyBody(bodyInterface_, id);
        bodyIds_ = null;
        bodiesOwned_ = null;
        poses_ = null;
        foreach (s; shapes_)
            synchronized (updateLock_)
                JPH_Shape_Destroy(cast(JPH_Shape*) s);
        shapes_ = null;
        ctx_.log.clear();
    }

    override void dispose()
    {
        clearScene();
        if (listener_ !is null)
        {
            JPH_ContactListener_Destroy(listener_);
            listener_ = null;
        }
        if (jobs_ !is null)
        {
            JPH_JobSystem_Destroy(jobs_);
            jobs_ = null;
        }
        JPH_PhysicsSystem_Destroy(system_);
        system_ = null;
        bodyInterface_ = null;
        ctx_.log.dispose();
    }
}

PhysWorld createPhysWorld()
{
    // Мир держит тела, шарниры и кэш поз в GC-контейнерах, поэтому New! (dlib)
    // недопустим: его память сборщик не сканирует, и `bodyIds_` уносится первым.
    return new JoltPhysWorld();
}
