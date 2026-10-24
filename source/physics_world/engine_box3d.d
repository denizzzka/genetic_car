/**
 * Бэкенд физики на Box3D (Box3D 1.2, D-биндинги `box3d`).
 *
 * Наружу отдаёт тот же `PhysWorld`, что и `physics_world.engine_jolt`, в
 * car-координатах. Оси Box3D (вверх вдоль Y) наружу не выходят.
 *
 * Владение телами и формами ведёт мир: Box3D отдаёт тела и шарниры
 * идентификаторами, а не объектами с деструктором, поэтому списки мира
 * заменяют `dlib.Owner`. Геометрия (hull-ы, сетки, heightfield-ы) - наша:
 * Box3D её не клонирует целиком и держит указатели до гибели формы.
 */
module physics_world.engine_box3d;

import std.algorithm: max, min;
import std.exception: enforce;
import std.math: PI, cos, sin;

import dlib.math.utils: clamp;
import dlib.math.vector;
import dlib.math.quaternion;

import box3d;

import physics_world.contactlog: ContactLog;
import physics_world.engine;

/// Гравитация мира, к которой приводится модуль из `PhysBody.gravity`.
enum float box3dDefaultGravity = 9.81f;

/// Подшаги внутри одного `step`: Box3D разбивает dt сам, и четыре подшага
/// дают устойчивые шарниры при шаге 30 Гц.
private enum int innerSteps = 4;

/// Мотор руля: частота и затухание позиционной пружины. Частота вместо
/// жёсткости не зависит от массы рычага.
private enum float steerMotorFrequency = 4.0f;
private enum float steerMotorDamping = 1.0f;

/* ------------------------------------------------------------------ *
 * Перевод осей
 *
 * (x, y, z)каркас → (x, z, −y)box3d: мир Box3D - это геометрия каркаса,
 * повёрнутая вокруг X на −90°, ровно как у Newton и Jolt.
 * ------------------------------------------------------------------ */

/// Выведено из образов осей базиса: LDC сворачивает `fromMatrix` в NaN.
private immutable Quaternionf carToBox3dQuat =
    Quaternionf(-0.70710678f, 0.0f, 0.0f, 0.70710678f);

private b3Vec3 toB3(const vec3 v)
{
    return b3Vec3(v.x, v.y, v.z);
}

private vec3 fromB3(const b3Vec3 v)
{
    return vec3(v.x, v.y, v.z);
}

private b3Quat toB3(const Quaternionf q)
{
    return b3Quat(b3Vec3(q.x, q.y, q.z), q.w);
}

private Quaternionf fromB3(const b3Quat q)
{
    return Quaternionf(q.v.x, q.v.y, q.v.z, q.s);
}

/* ------------------------------------------------------------------ *
 * Правила контактов
 *
 * Ключ - пара ролей. Отказ паре даёт кастомный фильтр Box3D, сцепление и
 * упругость - колбэки смешивания материалов; и то и другое получает роли
 * через `userMaterialId` формы.
 * ------------------------------------------------------------------ */

private struct RoleRule
{
    ContactResponse response = ContactResponse.resolve;
    float friction = 0.5f;
    float kineticFriction = 0.5f;
    float elasticity = 0.0f;
}

private struct ContactCtx
{
    /// Роли - ordinal-члены, индексируем таблицу ими самими.
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

    /// Модели нужны касания балки и пары под правилом `resolveAndReport`.
    private void report(const PhysBody a, const PhysBody b)
    {
        if (a.role == BodyRole.beam || b.role == BodyRole.beam
            || ruleFor(rules, a.role, b.role).response
                == ContactResponse.resolveAndReport)
            log.add(cast(PhysBody) a, cast(PhysBody) b);
    }
}

/// Колбэки Box3D не носят пользовательский контекст, поэтому таблица правил
/// шага живёт в TLS: шаг идёт на вызывающем потоке, а миры параллельных
/// заездов живут на разных потоках.
private ContactCtx* stepCtx_;

private extern(C) nothrow float box3dFriction(float frictionA,
    ulong materialA, float frictionB, ulong materialB)
{
    try
    {
        auto ctx = stepCtx_;
        if (ctx is null)
            return 0.5f;
        return ctx.ruleFor(ctx.rules, cast(BodyRole) materialA,
            cast(BodyRole) materialB).friction;
    }
    catch (Throwable)
    {
        return 0.5f;
    }
}

private extern(C) nothrow float box3dRestitution(float restitutionA,
    ulong materialA, float restitutionB, ulong materialB)
{
    try
    {
        auto ctx = stepCtx_;
        if (ctx is null)
            return 0.0f;
        return ctx.ruleFor(ctx.rules, cast(BodyRole) materialA,
            cast(BodyRole) materialB).elasticity;
    }
    catch (Throwable)
    {
        return 0.0f;
    }
}

private extern(C) nothrow bool box3dFilter(b3ShapeId shapeA,
    b3ShapeId shapeB, void* context)
{
    try
    {
        auto ctx = cast(ContactCtx*) context;
        auto a = physOf(shapeA);
        auto b = physOf(shapeB);
        if (ctx is null || a is null || b is null)
            return true;
        return !ctx.ignored(a.role, b.role);
    }
    catch (Throwable)
    {
        return true;
    }
}

/* ------------------------------------------------------------------ *
 * Формы
 *
 * Форма - набор выпуклых кусков и, для статической геометрии, владелец сетки
 * или поля высот. Box3D клонирует hull при создании формы, но сетку и
 * heightfield держит указателем, поэтому они живут до `destroyShape`.
 * ------------------------------------------------------------------ */

private struct HullPart
{
    b3HullData* hull;
    b3Transform transform = b3Transform_identity;
    /// Куску нужен собственный разворот: труба покрышки из брусьев.
    bool transformed;
}

private final class Shape: PhysShape
{
    HullPart[] hulls;
    b3MeshData* mesh;
    b3HeightFieldData* heightField;

    void dispose()
    {
        foreach (ref p; hulls)
            if (p.hull !is null)
            {
                b3DestroyHull(p.hull);
                p.hull = null;
            }
        hulls = null;
        if (mesh !is null)
        {
            b3DestroyMesh(mesh);
            mesh = null;
        }
        if (heightField !is null)
        {
            b3DestroyHeightField(heightField);
            heightField = null;
        }
    }
}

/// Восемь углов коробки в локальных осях тела.
private b3Vec3[8] boxPoints(const vec3 halfExtent)
{
    b3Vec3[8] points;
    foreach (i; 0 .. 8)
        points[i] = b3Vec3((i & 1) ? halfExtent.x : -halfExtent.x,
            (i & 2) ? halfExtent.y : -halfExtent.y,
            (i & 4) ? halfExtent.z : -halfExtent.z);
    return points;
}

/// Кольцо из двух окружностей: цилиндр или усечённый конус. Ось - локальная
/// Y, как требует модель.
private b3HullData* ringHull(float radiusBottom, float radiusTop, float height)
{
    enum int sides = 24;
    b3Vec3[2 * sides] points;
    foreach (k; 0 .. sides)
    {
        const float angle = 2.0f * PI * k / sides;
        const float c = cos(angle);
        const float s = sin(angle);
        points[2 * k] = b3Vec3(radiusBottom * c, -0.5f * height,
            radiusBottom * s);
        points[2 * k + 1] = b3Vec3(radiusTop * c, 0.5f * height, radiusTop * s);
    }
    return b3CreateHull(points.ptr, points.length, points.length);
}

/* ------------------------------------------------------------------ *
 * Тело
 * ------------------------------------------------------------------ */

private struct BodyPose
{
    vec3 pos;
    Quaternionf rot;
}

final class Box3DPhysBody: PhysBody
{
    Box3DPhysWorld world_;

    b3BodyId id_;
    private PhysShape shape_;

    private BodyRole role_;
    private BodyMotion motion_;
    private size_t tag_;

    private bool collidable_ = true;
    private float mass_ = 1.0f;
    private float gravity_ = box3dDefaultGravity;
    private vec3 inertia_ = vec3(0.01f, 0.01f, 0.01f);
    private vec3 comOffset_ = vec3(0.0f, 0.0f, 0.0f);
    private float linearDamping_ = 0.0f;
    private vec3 angularDamping_ = vec3(0.0f, 0.0f, 0.0f);

    /// D инициализирует float в NaN, а кэш обязан стартовать с нуля.
    BodyPose pose_ = BodyPose(vec3(0.0f, 0.0f, 0.0f),
        Quaternionf(0.0f, 0.0f, 0.0f, 1.0f));

    private b3ShapeId[] shapeIds_;

    this(Box3DPhysWorld world, BodyRole role, BodyMotion motion, Shape shape,
        float mass)
    {
        world_ = world;
        role_ = role;
        motion_ = motion;
        shape_ = shape;
        mass_ = mass > 0.0f ? mass : 1.0f;
        inertia_ = vec3(mass_ * 0.01f, mass_ * 0.01f, mass_ * 0.01f);

        b3BodyDef def = b3DefaultBodyDef();
        final switch (motion_)
        {
            case BodyMotion.staticBody:
                def.type = b3BodyType.b3_staticBody;
                break;
            case BodyMotion.kinematicBody:
                def.type = b3BodyType.b3_kinematicBody;
                break;
            case BodyMotion.dynamicBody:
                def.type = b3BodyType.b3_dynamicBody;
                break;
        }
        def.position = toB3(world_.toEnginePos(pose_.pos));
        def.rotation = toB3(world_.toEngineRot(pose_.rot));
        def.linearDamping = linearDamping_;
        def.angularDamping = 0.0f;
        def.gravityScale = gravity_ / box3dDefaultGravity;
        // Заезд длится доли секунды: сон рассинхронизирует раму с
        // кинематическими балками, которые продолжают за ней идти.
        def.enableSleep = false;
        def.isAwake = true;
        def.userData = cast(void*) this;

        id_ = b3CreateBody(world_.world_, &def);
        enforce(id_.index1 != 0, "Box3D не создал тело");
        attachShapes(shape, motion);
        applyMass();
        world_.track(this);
    }

    private void attachShapes(Shape shape, BodyMotion motion)
    {
        b3ShapeDef def = b3DefaultShapeDef();
        // Массу задаёт модель через `b3Body_SetMassData`, а не плотность.
        def.density = 0.0f;
        def.updateBodyMass = false;
        def.invokeContactCreation = false;
        // Сцепление и упругость приходят из правил ролей в колбэках
        // смешивания, поэтому у материала они нулевые.
        def.baseMaterial = b3DefaultSurfaceMaterial();
        def.baseMaterial.friction = 0.0f;
        def.baseMaterial.restitution = 0.0f;
        def.baseMaterial.userMaterialId = cast(ulong) role_;
        def.filter = filterOf(collidable_);
        def.enableCustomFiltering = true;
        def.enableContactEvents = motion != BodyMotion.staticBody;
        def.isSensor = role_ == BodyRole.beam;
        // События дают только пары, где флаг стоит у обоих: балка видит
        // колесо лишь тогда, когда оно тоже просит события.
        def.enableSensorEvents = true;
        def.userData = cast(void*) this;

        foreach (ref part; shape.hulls)
        {
            b3ShapeId id;
            if (part.transformed)
                id = b3CreateTransformedHullShape(id_, &def, part.hull,
                    part.transform, b3Vec3_one);
            else
                id = b3CreateHullShape(id_, &def, part.hull);
            enforce(id.index1 != 0, "Box3D не создал выпуклую форму");
            shapeIds_ ~= id;
        }
        if (shape.mesh !is null)
        {
            const b3ShapeId id = b3CreateMeshShape(id_, &def, shape.mesh,
                b3Vec3_one);
            enforce(id.index1 != 0, "Box3D не создал сетку");
            shapeIds_ ~= id;
        }
        if (shape.heightField !is null)
        {
            const b3ShapeId id = b3CreateHeightFieldShape(id_, &def,
                shape.heightField);
            enforce(id.index1 != 0, "Box3D не создал поле высот");
            shapeIds_ ~= id;
        }
    }

    private static b3Filter filterOf(bool collidable)
    {
        b3Filter filter = b3DefaultFilter();
        if (!collidable)
        {
            filter.categoryBits = 0;
            filter.maskBits = 0;
        }
        return filter;
    }

    /// Массовые свойства модели: главные моменты в локальных осях.
    private void applyMass()
    {
        if (motion_ != BodyMotion.dynamicBody || id_.index1 == 0)
            return;
        b3MassData data;
        data.mass = mass_;
        data.center = toB3(comOffset_);
        data.inertia = b3Matrix3(
            b3Vec3(inertia_.x, 0.0f, 0.0f),
            b3Vec3(0.0f, inertia_.y, 0.0f),
            b3Vec3(0.0f, 0.0f, inertia_.z));
        b3Body_SetMassData(id_, data);
    }

    private void pushPose()
    {
        if (id_.index1 == 0)
            return;
        b3Body_SetTransform(id_, toB3(world_.toEnginePos(pose_.pos)),
            toB3(world_.toEngineRot(pose_.rot)));
    }

    /// Поза из движка в кэш: после шага геттеры видят результат сразу.
    void readBackPose()
    {
        if (id_.index1 == 0)
            return;
        const b3Transform t = b3Body_GetTransform(id_);
        pose_.pos = world_.toCarPos(fromB3(t.p));
        pose_.rot = world_.toCarRot(fromB3(t.q));
    }

    /// Поза в осях движка: heightfield ставится без разворота в car-углы.
    void setEngineTransform(b3Vec3 position, b3Quat rotation)
    {
        b3Body_SetTransform(id_, position, rotation);
        pose_.pos = world_.toCarPos(fromB3(position));
        pose_.rot = world_.toCarRot(fromB3(rotation));
    }

    private void destroy()
    {
        if (id_.index1 == 0)
            return;
        b3DestroyBody(id_);
        id_ = b3BodyId.init;
    }

    void dispose()
    {
        destroy();
        world_.untrack(this);
    }

    override PhysWorld physWorld() @property { return world_; }
    override BodyRole role() @property const { return role_; }
    override PhysShape shape() @property { return shape_; }
    override void tag(size_t t) @property { tag_ = t; }
    override size_t tag() @property const { return tag_; }

    override Vector3f worldPosition() @property { return pose_.pos; }
    override Quaternionf worldRotation() @property { return pose_.rot; }

    /// Перевод осей - честная смена базиса, поправки к повороту нет.
    override Quaternionf worldFrameRotation() @property { return pose_.rot; }

    override void worldRotation(Quaternionf carRot) @property
    {
        // Вокруг текущего центра масс: иначе тело уедет из-под колёс.
        const vec3 com = worldCenterOfMass;
        pose_.pos = com + carRot.rotate(pose_.pos - com);
        pose_.rot = carRot;
        pushPose();
    }

    override void worldTransform(const vec3 carPos, const Quaternionf carRot) @property
    {
        pose_.pos = carPos;
        pose_.rot = carRot;
        pushPose();
    }

    override void setWorldPosition(const vec3 carPos)
    {
        pose_.pos = carPos;
        pushPose();
    }

    override Vector3f velocity() @property
    {
        return world_.toCarDir(fromB3(b3Body_GetLinearVelocity(id_)));
    }

    override void velocity(Vector3f v) @property
    {
        b3Body_SetLinearVelocity(id_, toB3(world_.toEngineDir(v)));
    }

    override Vector3f angularVelocity() @property
    {
        return world_.toCarDir(fromB3(b3Body_GetAngularVelocity(id_)));
    }

    override void angularVelocity(Vector3f w) @property
    {
        b3Body_SetAngularVelocity(id_, toB3(world_.toEngineDir(w)));
    }

    override Vector3f worldCenterOfMass() @property
    {
        return world_.toCarPos(fromB3(b3Body_GetWorldCenterOfMass(id_)));
    }

    override void localCenterOfMass(Vector3f offset) @property
    {
        comOffset_ = offset;
        applyMass();
    }

    override void mass(float m) @property
    {
        mass_ = m > 0.0f ? m : 1.0f;
        applyMass();
    }

    override void inertia(Vector3f principal) @property
    {
        inertia_ = principal;
        applyMass();
    }

    override void gravity(float accel) @property
    {
        gravity_ = accel;
        b3Body_SetGravityScale(id_, accel / box3dDefaultGravity);
    }

    override void linearDamping(float damping) @property
    {
        linearDamping_ = damping;
        b3Body_SetLinearDamping(id_, damping);
    }

    override void angularDamping(Vector3f damping) @property
    {
        // Box3D не различает демпфирование по осям: берётся максимум, иначе
        // рама теряет энергию быстрее, чем на Newton.
        angularDamping_ = damping;
        b3Body_SetAngularDamping(id_,
            max(damping.x, max(damping.y, damping.z)));
    }

    override void collidable(bool on) @property
    {
        collidable_ = on;
        const b3Filter filter = filterOf(on);
        foreach (id; shapeIds_)
            b3Shape_SetFilter(id, filter, false);
    }

    override void addForce(Vector3f f)
    {
        b3Body_ApplyForceToCenter(id_, toB3(world_.toEngineDir(f)), true);
    }

    override void addTorque(Vector3f t)
    {
        b3Body_ApplyTorque(id_, toB3(world_.toEngineDir(t)), true);
    }

    override void syncPose()
    {
        readBackPose();
    }
}

/* ------------------------------------------------------------------ *
 * Шарниры
 * ------------------------------------------------------------------ */

private interface Box3DJoint
{
    PhysBody bodyA() @property;
    PhysBody bodyB() @property;
    void dispose();
}

/// Ось колеса: револьверный шарнир вокруг локальной Y ступицы. Три линейных
/// ряда держат точку, два угловых - наклон оси, спин свободен.
private final class AxleJoint: PhysAxleJoint, Box3DJoint
{
    private PhysBody physA_, physB_;
    private b3JointId id_;

    this(PhysBody wheel, PhysBody master, const vec3 pivotMaster)
    {
        auto w = cast(Box3DPhysBody) wheel;
        auto m = cast(Box3DPhysBody) master;
        physA_ = wheel;
        physB_ = master;

        // Ось - живая ось ступицы: рама на момент сборки могла быть повёрнута.
        const vec3 axleCar = w.pose_.rot.rotate(Vector3f(0.0f, 1.0f, 0.0f));
        const vec3 pointCar = m.pose_.pos + m.pose_.rot.rotate(pivotMaster);
        const b3Transform frame = m.world_.frameFor(pointCar, axleCar);

        b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
        def.base.bodyIdA = m.id_;
        def.base.bodyIdB = w.id_;
        def.base.localFrameA = b3InvMulTransforms(b3Body_GetTransform(m.id_),
            frame);
        def.base.localFrameB = b3InvMulTransforms(b3Body_GetTransform(w.id_),
            frame);
        def.base.collideConnected = false;
        id_ = b3CreateRevoluteJoint(m.world_.world_, &def);
        enforce(b3Joint_IsValid(id_), "Box3D не создал ось колеса");
    }

    override PhysBody bodyA() @property { return physA_; }
    override PhysBody bodyB() @property { return physB_; }

    void dispose()
    {
        if (id_.index1 == 0)
            return;
        b3DestroyJoint(id_, true);
        id_ = b3JointId.init;
    }
}

/// Рулевой рычаг: тот же револьверный шарнир вокруг оси стойки, но с
/// позиционной пружиной и пределом хода. Ось разворачивается к car-up, чтобы
/// положительный угол означал поворот вправо, как `courseYaw` модели.
private final class SteerJoint: PhysSteerJoint, Box3DJoint
{
    private PhysBody physA_, physB_;
    private b3JointId id_;
    private float targetYaw_ = 0.0f;
    private float limit_;

    this(PhysBody steer, PhysBody master, const vec3 pivotSteer,
        const vec3 pivotMaster, const vec3 armUp, float limit)
    {
        auto s = cast(Box3DPhysBody) steer;
        auto m = cast(Box3DPhysBody) master;
        physA_ = steer;
        physB_ = master;
        limit_ = limit;

        vec3 axisCar = s.pose_.rot.rotate(armUp);
        if (axisCar.z < 0.0f)
            axisCar = -axisCar;
        const vec3 pointCar = m.pose_.pos + m.pose_.rot.rotate(pivotMaster);
        const b3Transform frame = m.world_.frameFor(pointCar, axisCar);

        b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
        def.base.bodyIdA = m.id_;
        def.base.bodyIdB = s.id_;
        def.base.localFrameA = b3InvMulTransforms(b3Body_GetTransform(m.id_),
            frame);
        def.base.localFrameB = b3InvMulTransforms(b3Body_GetTransform(s.id_),
            frame);
        def.base.collideConnected = false;
        def.enableSpring = true;
        def.hertz = steerMotorFrequency;
        def.dampingRatio = steerMotorDamping;
        def.targetAngle = 0.0f;
        def.enableLimit = true;
        def.lowerAngle = -limit;
        def.upperAngle = limit;
        id_ = b3CreateRevoluteJoint(m.world_.world_, &def);
        enforce(b3Joint_IsValid(id_), "Box3D не создал шарнир руля");
    }

    override PhysBody bodyA() @property { return physA_; }
    override PhysBody bodyB() @property { return physB_; }

    override void targetYaw(float yaw) @property
    {
        targetYaw_ = clamp(yaw, -limit_, limit_);
        b3RevoluteJoint_SetTargetAngle(id_, targetYaw_);
    }

    override float targetYaw() @property const { return targetYaw_; }

    override float yawNow() @property const
    {
        return b3RevoluteJoint_GetAngle(id_);
    }

    void dispose()
    {
        if (id_.index1 == 0)
            return;
        b3DestroyJoint(id_, true);
        id_ = b3JointId.init;
    }
}

/// Тело по идентификатору формы: Box3D хранит ссылку на него в userData.
private PhysBody physOf(b3ShapeId id)
{
    void* raw = b3Shape_GetUserData(id);
    if (raw is null)
        return null;
    return cast(PhysBody)(cast(Box3DPhysBody) raw);
}

/* ------------------------------------------------------------------ *
 * Мир
 * ------------------------------------------------------------------ */

final class Box3DPhysWorld: PhysWorld
{
    b3WorldId world_;

    ContactCtx ctx_;

    /// Всё, что мир отдал за заезд: пул миры не уничтожает, поэтому без
    /// этих списков они копились бы до гибели мира.
    private Shape[] shapes_;
    private Box3DPhysBody[] bodiesOwned_;
    private Box3DJoint[] joints_;

    /// Геометрия для снимков Box3D: образы ссылаются на неё по id.
    private b3SnapshotStore* snapshotStore_;

    this()
    {
        b3WorldDef def = b3DefaultWorldDef();
        def.gravity = b3Vec3(0.0f, -box3dDefaultGravity, 0.0f);
        // Заезд живёт доли секунды: сон только рассинхронизирует тела.
        def.enableSleep = false;
        def.enableContinuous = true;
        // Мир на вызывающем потоке: воркеры Box3D вне GC-рантайма.
        def.workerCount = 1;
        world_ = b3CreateWorld(&def);
        enforce(b3World_IsValid(world_), "Box3D не создал мир");
        b3World_SetFrictionCallback(world_, &box3dFriction);
        b3World_SetRestitutionCallback(world_, &box3dRestitution);
        b3World_SetCustomFilterCallback(world_, &box3dFilter,
            cast(void*) &ctx_);
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

    override vec3 toCarPos(const vec3 box3dPos)
    {
        return vec3(box3dPos.x, -box3dPos.z, box3dPos.y);
    }

    override Quaternionf toEngineRot(const Quaternionf carRot)
    {
        Quaternionf c = carToBox3dQuat;
        return c * carRot;
    }

    override Quaternionf toCarRot(const Quaternionf box3dRot)
    {
        Quaternionf c = carToBox3dQuat;
        return c.conj * box3dRot;
    }

    override vec3 toEngineDir(const vec3 carDir)
    {
        return vec3(carDir.x, carDir.z, -carDir.y);
    }

    override vec3 toCarDir(const vec3 box3dDir)
    {
        Quaternionf c = carToBox3dQuat;
        return c.conj.rotate(box3dDir);
    }

    /// Мировая поза шарнира: точка в car-координатах, ось - его локальная Z.
    b3Transform frameFor(const vec3 carPoint, const vec3 carAxis)
    {
        const vec3 axis = toEngineDir(carAxis).normalized;
        return b3Transform(toB3(toEnginePos(carPoint)),
            b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, toB3(axis)));
    }

    /// Мир Box3D считается на вызывающем потоке: воркеров у него нет.
    override void useCallingThread()
    {
    }

    override void step(double dt)
    {
        ctx_.log.clear();
        stepCtx_ = &ctx_;
        scope (exit) stepCtx_ = null;
        b3World_Step(world_, cast(float) dt, innerSteps);
        processEvents();
        foreach (b; bodiesOwned_)
            b.readBackPose();
    }

    private void processEvents()
    {
        const b3SensorEvents sensors = b3World_GetSensorEvents(world_);
        foreach (i; 0 .. sensors.beginCount)
        {
            const e = sensors.beginEvents[i];
            reportPair(e.sensorShapeId, e.visitorShapeId);
        }
        const b3ContactEvents contacts = b3World_GetContactEvents(world_);
        foreach (i; 0 .. contacts.beginCount)
        {
            const e = contacts.beginEvents[i];
            reportPair(e.shapeIdA, e.shapeIdB);
        }
    }

    private void reportPair(b3ShapeId shapeA, b3ShapeId shapeB)
    {
        auto a = physOf(shapeA);
        auto b = physOf(shapeB);
        if (a is null || b is null)
            return;
        ctx_.report(a, b);
    }

    override double updateRate() @property { return 30.0; }

    override const(ContactPair)[] contacts() const { return ctx_.log.pairs; }

    /// Box3D сохраняет всё: тела, формы, контакты с warm start, острова,
    /// таймеры сна и broad-phase. Восстановление бит-в-бит.
    override bool supportsSnapshot() @property { return true; }

    override ubyte[] saveSnapshot()
    {
        ubyte* img;
        const int size = b3World_SaveSnapshot(world_, snapshotStore(), &img);
        if (size <= 0 || img is null)
            return null;
        auto data = new ubyte[size];
        data[] = img[0 .. size];
        b3FreeSnapshot(img, size);
        return data;
    }

    override bool restoreSnapshot(const ubyte[] image)
    {
        if (image.length == 0)
            return false;
        if (!b3World_RestoreSnapshot(world_, snapshotStore(), image.ptr,
            cast(int) image.length))
            return false;
        foreach (b; bodiesOwned_)
            b.readBackPose();
        ctx_.log.clear();
        return true;
    }

    private b3SnapshotStore* snapshotStore()
    {
        if (snapshotStore_ is null)
            snapshotStore_ = b3CreateSnapshotStore();
        return snapshotStore_;
    }

    /// Формы мира: Box3D клонирует hull, но сетку и поле высот держит
    /// указателем, поэтому освобождать их нужно вместе с формой.
    private Shape keepShape(Shape shape)
    {
        shapes_ ~= shape;
        return shape;
    }

    override PhysShape boxShape(const vec3 halfExtent)
    {
        auto points = boxPoints(halfExtent);
        b3HullData* hull = b3CreateHull(points.ptr, points.length,
            points.length);
        enforce(hull !is null, "Box3D отверг коробку");
        auto shape = new Shape();
        shape.hulls ~= HullPart(hull);
        return keepShape(shape);
    }

    override PhysShape cylinderShape(float radius1, float radius2, float height)
    {
        b3HullData* hull = ringHull(radius1, radius2, height);
        enforce(hull !is null, "Box3D отверг цилиндр");
        auto shape = new Shape();
        shape.hulls ~= HullPart(hull);
        return keepShape(shape);
    }

    /// Покрышка колеса: труба из `tubeSegments` брусьев вокруг оси Y.
    override PhysShape tubeShape(float outerRadius, float innerRadius, float height)
    {
        auto shape = new Shape();
        foreach (k; 0 .. tubeSegments)
        {
            const float angle = 2.0f * PI * k / tubeSegments;
            const auto segment = tubeSegment(outerRadius, innerRadius, height,
                angle);
            auto points = boxPoints(segment.halfExtent);
            b3HullData* hull = b3CreateHull(points.ptr, points.length,
                points.length);
            enforce(hull !is null, "Box3D отверг брус покрышки");
            const b3Transform transform = b3Transform(
                toB3(segment.center),
                b3MakeQuatFromAxisAngle(b3Vec3_axisY, angle));
            shape.hulls ~= HullPart(hull, transform, true);
        }
        return keepShape(shape);
    }

    override PhysShape triangleMeshShape(const float[] vertices,
        const float[] normals, const uint[] indices)
    {
        const uint nv = cast(uint) (vertices.length / 3);
        auto vs = new b3Vec3[nv];
        foreach (i; 0 .. nv)
        {
            const vec3 p = vec3(vertices[i * 3], vertices[i * 3 + 1],
                vertices[i * 3 + 2]);
            vs[i] = toB3(toEnginePos(p));
        }
        auto tris = new int[indices.length];
        foreach (i, v; indices)
            tris[i] = cast(int) v;

        b3MeshDef def;
        def.vertices = vs.ptr;
        def.indices = tris.ptr;
        def.vertexCount = cast(int) nv;
        def.triangleCount = cast(int) (indices.length / 3);
        def.weldVertices = false;
        def.useMedianSplit = false;
        def.identifyEdges = false;
        b3MeshData* mesh = b3CreateMesh(&def, null, 0);
        enforce(mesh !is null, "Box3D отверг сетку");
        auto shape = new Shape();
        shape.mesh = mesh;
        return keepShape(shape);
    }

    /// Ровная земля: Box3D без плоскости, поэтому коробка с верхом на `topZ`.
    override PhysBody createFlatGround(float halfExtent, uint cells, float topZ)
    {
        enum float thickness = 1.0f;
        auto shape = cast(Shape) boxShape(vec3(halfExtent, halfExtent,
            thickness));
        auto b = createBody(BodyRole.ground, BodyMotion.staticBody, shape, 0.0f);
        b.setWorldPosition(vec3(0.0f, 0.0f, topZ - thickness));
        b.syncPose();
        return b;
    }

    /// Окно terrain: сетка высот приходит в car-координатах и ложится в
    /// локальные оси поля (x - вправо, z - вперёд, высота - вдоль y).
    override PhysBody createHeightfieldGround(const float[] heights, uint dims,
        float cell, const vec3 corner)
    {
        const size_t n = cast(size_t) dims * dims;
        auto samples = new float[n];
        float lo = float.max;
        float hi = -float.max;
        foreach (i; 0 .. n)
        {
            const float h = heights[i];
            samples[i] = h;
            lo = min(lo, h);
            hi = max(hi, h);
        }

        b3HeightFieldDef def;
        def.heights = samples.ptr;
        def.materialIndices = null;
        def.scale = b3Vec3(cell, 1.0f, cell);
        def.countX = cast(int) dims;
        def.countZ = cast(int) dims;
        def.globalMinimumHeight = lo;
        def.globalMaximumHeight = hi;
        def.clockwiseWinding = false;
        b3HeightFieldData* field = b3CreateHeightField(&def);
        enforce(field !is null, "Box3D отверг поле высот");

        auto shape = new Shape();
        shape.heightField = field;
        keepShape(shape);
        auto b = createBody(BodyRole.ground, BodyMotion.staticBody, shape, 0.0f);
        // Поле уже лежит в осях движка, поэтому тело ставится без разворота:
        // локальные x/z совпадают с right/forward каркаса.
        auto jb = cast(Box3DPhysBody) b;
        jb.setEngineTransform(b3Vec3(corner.x, 0.0f, -corner.y),
            b3Quat_identity);
        return b;
    }

    override PhysBody createBody(BodyRole role, BodyMotion motion,
        PhysShape shape, float mass)
    {
        auto s = cast(Shape) shape;
        enforce(s !is null, "форма выдана не этому бэкенду");
        return new Box3DPhysBody(this, role, motion, s, mass);
    }

    override void destroyBody(PhysBody body)
    {
        auto b = cast(Box3DPhysBody) body;
        if (b !is null)
            b.dispose();
    }

    override void destroyShape(PhysShape shape)
    {
        auto s = cast(Shape) shape;
        if (s is null)
            return;
        untrackShape(s);
        s.dispose();
    }

    override PhysAxleJoint newAxleJoint(PhysBody wheel, PhysBody master,
        const vec3 pivotMaster)
    {
        auto j = new AxleJoint(wheel, master, pivotMaster);
        joints_ ~= j;
        return j;
    }

    override PhysSteerJoint newSteerJoint(PhysBody steer, PhysBody master,
        const vec3 pivotSteer, const vec3 pivotMaster, const vec3 armUp,
        float limit)
    {
        auto j = new SteerJoint(steer, master, pivotSteer, pivotMaster, armUp,
            limit);
        joints_ ~= j;
        return j;
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

    void track(Box3DPhysBody body)
    {
        bodiesOwned_ ~= body;
    }

    void untrack(Box3DPhysBody body)
    {
        foreach (i, b; bodiesOwned_)
            if (b is body)
            {
                bodiesOwned_[i] = bodiesOwned_[$ - 1];
                bodiesOwned_.length = bodiesOwned_.length - 1;
                return;
            }
    }

    private void untrackShape(Shape shape)
    {
        foreach (i, s; shapes_)
            if (s is shape)
            {
                shapes_[i] = shapes_[$ - 1];
                shapes_.length = shapes_.length - 1;
                return;
            }
    }

    override void clearScene()
    {
        // Порядок обязателен: шарнир ссылается на тела, а тело - на форму.
        foreach (j; joints_)
            j.dispose();
        joints_ = null;
        foreach (b; bodiesOwned_)
            b.dispose();
        bodiesOwned_ = null;
        foreach (s; shapes_)
            s.dispose();
        shapes_ = null;
        if (snapshotStore_ !is null)
        {
            b3DestroySnapshotStore(snapshotStore_);
            snapshotStore_ = null;
        }
        ctx_.log.clear();
    }

    override void dispose()
    {
        clearScene();
        if (b3World_IsValid(world_))
            b3DestroyWorld(world_);
        world_ = b3WorldId.init;
        if (snapshotStore_ !is null)
        {
            b3DestroySnapshotStore(snapshotStore_);
            snapshotStore_ = null;
        }
        ctx_.log.dispose();
    }
}

PhysWorld createPhysWorld()
{
    // Мир держит тела и кэш поз в GC-контейнерах, поэтому New! (dlib)
    // недопустим: его память сборщик не сканирует.
    return new Box3DPhysWorld();
}
