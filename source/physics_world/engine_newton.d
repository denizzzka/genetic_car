/**
 * Бэкенд физики на Newton.
 *
 * Здесь и только здесь живут вещи, о которых модель машины знать не должна:
 * оси Newton (у него «вверх» вдоль Y, у каркаса — вдоль Z), группы
 * материалов, сенсорные колбэки C++, heightfield'ы и пользовательские
 * шарниры. Наружу отдаётся `PhysWorld` из `physics_world.engine`.
 */
module physics_world.engine_newton;

import std.algorithm : clamp;
import std.exception : enforce;
import std.math : PI, atan2, fabs;

import dlib.core.memory;
import dlib.core.ownership;
import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;
import dlib.math.transformation;

import bindbc.newton;
import dagon.core.event;
import dagon.ext.newton;

import physics_world.engine;

/* ------------------------------------------------------------------ *
 * Загрузка библиотеки
 * ------------------------------------------------------------------ */

/// Ньютоновская библиотека грузится один раз на процесс: bindbc resolve
/// символов динамической библиотеки не потокобезопасен, а заезды идут на пуле
/// воркеров. Гвард — ОТКРЫТЫЙ (не потоково-локальный) мьютекс.
private __gshared bool newtonLoaded_;
private __gshared Object newtonLoadLock_ = new Object();

void ensureNewtonLoaded()
{
    if (newtonLoaded_)
        return;
    synchronized (newtonLoadLock_)
    {
        if (!newtonLoaded_)
        {
            auto sup = loadNewton();
            enforce(sup == NewtonSupport.newton314,
                "Не загрузилась libnewton.so. В каталоге сборки должны лежать "
                ~ "libnewton.so, libdgCore.so, libdgPhysics.so, libdgNewtonAvx.so "
                ~ "(копируются из dagon:newton); при ручном запуске добавь их "
                ~ "в LD_LIBRARY_PATH.");
            newtonLoaded_ = true;
        }
    }
}

/* ------------------------------------------------------------------ *
 * Перевод осей
 *
 * (x, y, z)каркас → (x, z, −y)newton: мир Newton — это геометрия каркаса,
 * повёрнутая вокруг X на −90°.
 * ------------------------------------------------------------------ */

/// Выведено из образов осей базиса: литерал, потому что LDC сворачивает
/// `fromMatrix` в NaN на этапе компиляции.
immutable Quaternionf carToNewtonQuat =
    Quaternionf(-0.70710678f, 0.0f, 0.0f, 0.70710678f);

private vec3 toNewtonPos(const vec3 carPos)
{
    return vec3(carPos.x, carPos.z, -carPos.y);
}

private vec3 toCarPos(const vec3 newtonPos)
{
    return vec3(newtonPos.x, -newtonPos.z, newtonPos.y);
}

private Quaternionf toNewtonRot(const Quaternionf carRot)
{
    // dlib над кватернионами не const — работаем на копии.
    Quaternionf r = carToNewtonQuat;
    return r * carRot;
}

/// `body.rotation` — инверсия истинного поворота (её кэширует dagon), так
/// что наружу уходит `carToNewton⁻¹ · conj(rotation)`.
private Quaternionf toCarRot(const Quaternionf cachedNewtonRot)
{
    Quaternionf r = carToNewtonQuat;
    Quaternionf c = cachedNewtonRot;
    return r.conj * c.conj;
}

/// Вектор поворачивается тем же поворотом, что и тело: ось угловой скорости
/// живёт в мире, а не в системе тела.
private Vector3f toNewtonDir(const Vector3f carDir)
{
    Quaternionf q = carToNewtonQuat;
    return q.rotate(carDir);
}

private Vector3f toCarDir(const Vector3f newtonDir)
{
    Quaternionf q = carToNewtonQuat;
    return q.conj.rotate(newtonDir);
}

private Matrix4x4f carBodyMatrix(const vec3 carPos, const Quaternionf carRot)
{
    return translationMatrix(toNewtonPos(carPos)) * toNewtonRot(carRot).toMatrix4x4;
}

/// Ускорение свободного падения в координатах Newton: вниз вдоль −Y.
private Vector3f newtonGravity(float accel)
{
    return Vector3f(0.0f, -accel, 0.0f);
}

/* ------------------------------------------------------------------ *
 * Контакты за шаг
 *
 * Пишут сюда колбэки C++, читает модель. Живёт отдельно от мира, чтобы
 * колбэкам не нужно было знать тип мира (они приходят из движка и видят
 * только указатели на тела).
 *
 * Журнал хранится ПО ЗНАЧЕНИЮ в `NewtonPhysWorld` и держит буфер пар в
 * dlib-памяти. Пока он был GC-классом, единственные ссылки на него жили
 * в dlib-указателях (мир и сенсорные тела), а GC такую память не
 * обходит: под давлением аллокаций журнал уходил в free-list и
 * переписывался чужыми данными прямо во время заезда — зависание и
 * SIGSEGV в разборе контактов.
 * ------------------------------------------------------------------ */

struct ContactLog
{
    /// Буфер пар в dlib-памяти: GC-объекты из мира его не увидят.
    private ContactPair[] buf_;
    private size_t len_;

    const(ContactPair)[] pairs() const { return buf_[0 .. len_]; }

    /// Повторы гасим: сенсорный диспетчер и материальный колбэк могут
    /// сообщить одну пару дважды, а вердикту важно лишь «было». Контактов на
    /// шаг единицы, поэтому поиск линейный.
    void add(PhysBody a, PhysBody b)
    {
        if (a is null || b is null || a is b)
            return;
        foreach (i; 0 .. len_)
            if (buf_[i].a is a && buf_[i].b is b)
                return;
        if (len_ == buf_.length)
            grow();
        buf_[len_++] = ContactPair(a, b);
    }

    void clear()
    {
        len_ = 0;
    }

    void dispose()
    {
        freeBuf();
        len_ = 0;
    }

    private void grow()
    {
        const size_t cap = buf_.length == 0 ? 16 : buf_.length * 2;
        auto grown = New!(ContactPair[])(cap);
        foreach (i; 0 .. len_)
            grown[i] = buf_[i];
        freeBuf();
        buf_ = grown;
    }

    /// `Delete` у dlib читает заголовок размера перед указателем, поэтому
    /// на пустом массиве (контактов не было) он падает.
    private void freeBuf()
    {
        if (buf_ !is null)
            Delete(buf_);
        buf_ = null;
    }
}

/* ------------------------------------------------------------------ *
 * Группы материалов
 *
 * Роль тела lowered в группу: земля получает собственную группу (сцепление
 * покрышек берётся только с неё), колесо и рама живут в default, балки — в
 * sensor, где контакт виден, но не толкает.
 * ------------------------------------------------------------------ */

final class SoilWorld : NewtonPhysicsWorld
{
    int soilGroupId;

    this(EventManager eventManager, Owner o)
    {
        super(eventManager, o);
        soilGroupId = createGroupId();
        // Балка о грунт: контакт гасится, импульс грунту не передаётся.
        NewtonMaterialSetCollisionCallback(newtonWorld, sensorGroupId,
            soilGroupId, &ignorePair, null);
    }
}

private int groupOf(const SoilWorld w, const BodyRole role)
{
    final switch (role)
    {
        case BodyRole.ground: return w.soilGroupId;
        case BodyRole.master:
        case BodyRole.wheel:
        case BodyRole.obstacle: return w.defaultGroupId;
        case BodyRole.beam: return w.sensorGroupId;
    }
}

/* ------------------------------------------------------------------ *
 * Внутренняя обёртка тела
 *
 * dagon-класс, в котором Newton хранит обратную ссылку на тело: из сырого
 * `NewtonBody*` C++ отдаёт его нам, и по нему мы находим тело и журнал
 * контактов.
 * ------------------------------------------------------------------ */

final class NewtonSensorBody : NewtonRigidBody
{
    PhysBody phys;
    ContactLog* log;

    this(NewtonRigidBodyType bodyType, NewtonCollisionShape shape, float mass,
        NewtonPhysicsWorld world, Owner owner, ContactLog* log)
    {
        super(bodyType, shape, mass, world, owner);
        this.log = log;
    }
}

/// Контакты двух sensor-тел (узлы каркаса в одной точке) — не столкновение.
private extern(C) int ignorePair(const NewtonJoint*, dFloat, int)
{
    return 0;
}

private void reportJoint(const NewtonJoint* joint)
{
    auto a = cast(NewtonSensorBody) NewtonBodyGetUserData(
        NewtonJointGetBody0(joint));
    auto b = cast(NewtonSensorBody) NewtonBodyGetUserData(
        NewtonJointGetBody1(joint));
    if (a is null || b is null || a.phys is null || b.phys is null
        || a.log is null)
        return;
    a.log.add(a.phys, b.phys);
}

/// Снять контакт, чтобы тело не толкалось, и сообщить пару модели.
private extern(C) void stripAndReport(const NewtonJoint* joint, dFloat, int)
{
    void* next;
    for (void* c = NewtonContactJointGetFirstContact(joint); c; c = next)
    {
        next = NewtonContactJointGetNextContact(joint, c);
        NewtonContactJointRemoveContact(joint, c);
    }
    reportJoint(joint);
}

/// Рапортовать пару тел обычных групп (колесо × колесо).
private extern(C) void reportPairCallback(const NewtonJoint* joint, dFloat, int)
{
    reportJoint(joint);
}

/* ------------------------------------------------------------------ *
 * Tree-коллизия из треугольников: сетка строится напрямую, чтобы бэкенд не
 * зависел от графики dagon (меш — это просто вершины с нормалями).
 * ------------------------------------------------------------------ */

private final class TreeShape : NewtonCollisionShape
{
    this(NewtonPhysicsWorld world, const float[] vertices,
        const float[] normals, const uint[] indices)
    {
        super(world);
        NewtonMesh* mesh = NewtonMeshCreate(world.newtonWorld);
        NewtonMeshBeginBuild(mesh);
        foreach (i; 0 .. indices.length / 3)
        {
            NewtonMeshBeginFace(mesh);
            foreach (k; 0 .. 3)
            {
                const uint v = indices[i * 3 + k] * 3;
                NewtonMeshAddPoint(mesh, vertices[v], vertices[v + 1],
                    vertices[v + 2]);
                NewtonMeshAddNormal(mesh, normals[v], normals[v + 1],
                    normals[v + 2]);
            }
            NewtonMeshEndFace(mesh);
        }
        NewtonMeshEndBuild(mesh);
        newtonCollision = NewtonCreateTreeCollisionFromMesh(
            world.newtonWorld, mesh, 0);
        NewtonMeshDestroy(mesh);
    }
}

/* ------------------------------------------------------------------ *
 * Формы
 *
 * Формы движка — классы dagon, и они не реализуют `PhysShape`. Каст на
 * нереализуемый интерфейс вернул бы `null` в рантайме, поэтому форма — это
 * обёртка: наружу она непрозрачна, внутрь держит форму Newton.
 * ------------------------------------------------------------------ */

private final class Shape : PhysShape
{
    NewtonCollisionShape inner;

    this(NewtonCollisionShape inner)
    {
        this.inner = inner;
    }
}

/* ------------------------------------------------------------------ *
 * Heightfield'ы
 *
 * Newton 3.14 НЕ копирует height-данные: коллизия хранит на них указатели,
 * поэтому массивы живут всё время жизни коллизии.
 * ------------------------------------------------------------------ */

/// Плоская земля: ровная плоскость с нулём высот. Newton строит по size−1
/// клеток на сторону при аргументе size, а буферы ждёт размера size² — отсюда
/// лишняя строка и клетка.
private final class FlatHeightfield : NewtonCollisionShape
{
    private float[] elevations_;
    private ubyte[] attributes_;

    this(NewtonPhysicsWorld world, float halfExtent, uint cells)
    {
        super(world);
        const uint size = cells + 1;
        elevations_ = New!(float[])(cast(size_t)size * size);
        foreach (ref h; elevations_)
            h = 0.0f;
        attributes_ = New!(ubyte[])(cast(size_t)size * size);
        foreach (ref a; attributes_)
            a = 0;

        const float cell = 2.0f * halfExtent / cast(float)cells;
        newtonCollision = NewtonCreateHeightFieldCollision(world.newtonWorld,
            cast(int)size, cast(int)size, 1, // gridsDiagonals
            0, // elevationdatType: float
            elevations_.ptr, cast(char*) attributes_.ptr,
            1.0f, // verticalScale
            cell, cell, // horizontalScale по X и Z
            0); // shapeId
    }

    ~this()
    {
        // Порядок обязателен: Newton держит на высотах указатель, поэтому
        // коллизия должна умереть раньше буферов, а базовый деструктор
        // после обнуления указателя уже ничего не трогает.
        if (newtonCollision)
        {
            NewtonDestroyCollision(newtonCollision);
            newtonCollision = null;
        }
        Delete(elevations_);
        Delete(attributes_);
    }
}

/// Окно terrain: сетка высот приходит в car-координатах.
private final class WindowHeightfield : NewtonCollisionShape
{
    private float[] elevations_;
    private ubyte[] attributes_;

    this(NewtonPhysicsWorld world, const float[] heights, uint dims, float cell)
    {
        super(world);
        elevations_ = New!(float[])(heights.length);
        foreach (i, h; heights)
            elevations_[i] = h;
        attributes_ = New!(ubyte[])(cast(size_t)dims * dims);
        foreach (ref a; attributes_)
            a = 0;

        newtonCollision = NewtonCreateHeightFieldCollision(world.newtonWorld,
            cast(int)dims, cast(int)dims, 1,
            0,
            elevations_.ptr, cast(char*) attributes_.ptr,
            1.0f,
            cell, cell,
            0);
    }

    ~this()
    {
        // Порядок обязателен: Newton держит на высотах указатель, поэтому
        // коллизия должна умереть раньше буферов, а базовый деструктор
        // после обнуления указателя уже ничего не трогает.
        if (newtonCollision)
        {
            NewtonDestroyCollision(newtonCollision);
            newtonCollision = null;
        }
        Delete(elevations_);
        Delete(attributes_);
    }
}

/* ------------------------------------------------------------------ *
 * Тело
 * ------------------------------------------------------------------ */

final class NewtonPhysBody : PhysBody
{
    NewtonSensorBody body_;
    NewtonPhysWorld world_;

    private BodyRole role_;
    private size_t tag_;

    this(NewtonPhysWorld w, BodyRole role, NewtonRigidBodyType bodyType,
        NewtonCollisionShape shape, float mass, Owner owner)
    {
        world_ = w;
        role_ = role;
        body_ = New!NewtonSensorBody(bodyType, shape, mass, w.newton, owner,
            &w.log_);
        body_.phys = this;
        body_.groupId = groupOf(w.newton, role);
        body_.dynamic = bodyType == NewtonRigidBodyType.Dynamic;
        // Заезд длится доли секунды: сон тела только рассинхронизирует раму с
        // кинематическими балками, которые продолжают за ней идти.
        body_.autoSleep = false;
        // Балка наблюдает землю, но не толкается: пара «sensor × грунт»
        // разбирается без передачи импульса (см. `stripAndReport`).
        body_.sensor = role == BodyRole.beam;
        if (body_.sensor)
            body_.sensorCallback = (NewtonRigidBody self, NewtonRigidBody other)
            {
                sensorDispatch(self, other);
            };
    }

    private static void sensorDispatch(NewtonRigidBody self, NewtonRigidBody other)
    {
        auto a = cast(NewtonSensorBody) self;
        auto b = cast(NewtonSensorBody) other;
        if (a is null || b is null || a.phys is null || b.phys is null
            || a.log is null)
            return;
        a.log.add(a.phys, b.phys);
    }

    override PhysWorld physWorld() @property { return world_; }
    override PhysShape shape() @property { return New!Shape(body_.collisionShape); }
    override BodyRole role() @property const { return role_; }
    override void tag(size_t t) @property { tag_ = t; }
    override size_t tag() @property const { return tag_; }

    override Vector3f worldPosition() @property { return toCarPos(body_.position.xyz); }
    override Quaternionf worldRotation() @property { return toCarRot(body_.rotation); }

    override Quaternionf worldFrameRotation() @property
    {
        // `toCarRot` оставляет в повороте постоянный разворот осей фантома;
        // домножаем на базис каркаса, и у неповёрнутого тела выходит тождество.
        Quaternionf r = carToNewtonQuat;
        return toCarRot(body_.rotation) * r;
    }

    override void worldRotation(Quaternionf carRot) @property
    {
        // Вокруг текущего центра масс: иначе тело уедет из-под привязанных
        // к нему колёс.
        body_.setTransformation(
            translationMatrix(toNewtonPos(body_.worldCenterOfMass))
            * toNewtonRot(carRot).toMatrix4x4);
        body_.update(0.0);
    }

    override void worldTransform(const vec3 carPos, const Quaternionf carRot) @property
    {
        body_.setTransformation(carBodyMatrix(carPos, carRot));
        body_.update(0.0);
    }

    override void setWorldPosition(const vec3 carPos)
    {
        body_.setTransformation(translationMatrix(toNewtonPos(carPos)));
        body_.update(0.0);
    }

    override Vector3f velocity() @property { return toCarDir(body_.velocity); }
    override void velocity(Vector3f v) @property { body_.velocity = toNewtonPos(v); }
    override Vector3f angularVelocity() @property { return toCarDir(body_.angularVelocity); }
    override void angularVelocity(Vector3f w) @property { body_.angularVelocity = toNewtonDir(w); }

    override Vector3f worldCenterOfMass() @property { return toCarPos(body_.worldCenterOfMass); }
    override void localCenterOfMass(Vector3f offset) @property { body_.centerOfMass = offset; }
    override void mass(float m) @property { body_.mass = m; }
    override void inertia(Vector3f principal) @property
    {
        body_.setMassMatrix(body_.mass, principal.x, principal.y, principal.z);
    }
    override void gravity(float accel) @property { body_.gravity = newtonGravity(accel); }
    override void linearDamping(float damping) @property { body_.linearDamping = damping; }
    override void angularDamping(Vector3f damping) @property { body_.angularDamping = damping; }
    override void collidable(bool on) @property { body_.collidable = on; }
    override void addForce(Vector3f f) { body_.addForce(toNewtonPos(f)); }
    override void addTorque(Vector3f t) { body_.addTorque(toNewtonDir(t)); }
    override void syncPose() { body_.update(0.0); }

}

/* ------------------------------------------------------------------ *
 * Шарниры
 * ------------------------------------------------------------------ */

/// Единица вектора или ноль: ось всегда определена, но страхуем /0.
private vec3 unitOrZero(vec3 v)
{
    const float l = v.length;
    return l < 1e-5f ? Vector3f(0.0f, 0.0f, 0.0f) : v / l;
}

/// Знаковый угол между `dir` и `cosDir` в плоскости с нормалью `sinDir`
/// (порт `dCustomJoint::CalculateAngle`).
private float calculateAngle(vec3 dir, vec3 cosDir, vec3 sinDir)
{
    const vec3 projectDir = dir - sinDir * dot(dir, sinDir);
    const float cosAngle = dot(projectDir, cosDir);
    const float sinAngle = dot(sinDir, cross(projectDir, cosDir));
    return atan2(sinAngle, cosAngle);
}

/// Ось колеса: три линейных ряда держат ступицу, два угловых гасят крен оси,
/// оставляя свободным вращение вокруг неё самой.
private final class AxleJoint : NewtonUserConstraint, PhysAxleJoint
{
    private NewtonRigidBody wheel_;
    private NewtonRigidBody master_;
    private vec3 pivotMasterLocal_;
    private vec3 masterPinLocal_;
    private vec3 masterUpLocal_;
    private vec3 masterRightLocal_;

    private PhysBody physA_, physB_;

    this(PhysBody wheel, PhysBody master, const vec3 pivotMaster)
    {
        auto w = cast(NewtonPhysBody) wheel;
        auto m = cast(NewtonPhysBody) master;
        physA_ = w;
        physB_ = m;
        wheel_ = w.body_;
        master_ = m.body_;
        pivotMasterLocal_ = toNewtonPos(pivotMaster);
        super(master_.world, wheel_, master_, 6);

        // Ось в локальных координатах мастера: берём живую ось колеса, а не
        // кэш — мастер на момент сборки мог быть уже повёрнут.
        Matrix4x4f wm, mm;
        NewtonBodyGetMatrix(wheel_.newtonBody, wm.arrayof.ptr);
        NewtonBodyGetMatrix(master_.newtonBody, mm.arrayof.ptr);
        const vec3 axleWorld = wm.rotate(Vector3f(0.0f, 1.0f, 0.0f));
        Quaternionf masterTrue = Quaternionf.fromMatrix(mm).conj;
        masterPinLocal_ = masterTrue.conj.rotate(axleWorld);

        const vec3 tmp = fabs(masterPinLocal_.x) < 0.9f
            ? Vector3f(1.0f, 0.0f, 0.0f) : Vector3f(0.0f, 1.0f, 0.0f);
        masterUpLocal_ = unitOrZero(cross(masterPinLocal_, tmp));
        masterRightLocal_ = unitOrZero(cross(masterPinLocal_, masterUpLocal_));
    }

    override PhysBody bodyA() @property { return physA_; }
    override PhysBody bodyB() @property { return physB_; }

    override void submit(float timestep, int threadIndex)
    {
        // Живые матрицы на момент решения: кэш обёртки отстаёт.
        Matrix4x4f m0, m1;
        NewtonBodyGetMatrix(wheel_.newtonBody, m0.arrayof.ptr);
        NewtonBodyGetMatrix(master_.newtonBody, m1.arrayof.ptr);

        const vec3 pivot0 = Vector3f(0.0f, 0.0f, 0.0f) * m0;
        const vec3 pivot1 = pivotMasterLocal_ * m1;

        const vec3 front = m1.rotate(masterPinLocal_);
        const vec3 up = m1.rotate(masterUpLocal_);
        const vec3 right = m1.rotate(masterRightLocal_);

        addLinearRow(pivot0, pivot1, front);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, up);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, right);
        setRowStiffness(1.0f);

        const vec3 wheelPin = m0.rotate(Vector3f(0.0f, 1.0f, 0.0f));

        addAngularRow(calculateAngle(wheelPin, front, up), up);
        setRowStiffness(1.0f);
        addAngularRow(calculateAngle(wheelPin, front, right), right);
        setRowStiffness(1.0f);
    }
}

/// Рулевой рычаг: шарнир с приводом на рыскание относительно рамы. Кап ошибки
/// не даёт ненагруженной балке раскрутиться — правка ряда подпитывает своё
/// же измерение. За пределом хода жёсткий ряд возвращает рычаг к границе.
private final class SteerJoint : NewtonUserConstraint, PhysSteerJoint
{
    private NewtonRigidBody steer_;
    private NewtonRigidBody master_;
    private vec3 pivotSteerLocal_;
    private vec3 pivotMasterLocal_;
    private vec3 masterUpLocal_;
    private vec3 armUpLocal_;

    private PhysBody physA_, physB_;

    private float targetYaw_;
    private float limit_;
    private float errorCap_;

    this(PhysBody steer, PhysBody master, const vec3 pivotSteer,
        const vec3 pivotMaster, const vec3 armUp, float limit, float errorCap)
    {
        auto s = cast(NewtonPhysBody) steer;
        auto m = cast(NewtonPhysBody) master;
        physA_ = s;
        physB_ = m;
        steer_ = s.body_;
        master_ = m.body_;
        pivotSteerLocal_ = pivotSteer;
        pivotMasterLocal_ = toNewtonPos(pivotMaster);
        armUpLocal_ = armUp;
        masterUpLocal_ = Vector3f(0.0f, 1.0f, 0.0f);
        limit_ = limit;
        errorCap_ = errorCap;
        super(master_.world, steer_, master_, 8);
    }

    override PhysBody bodyA() @property { return physA_; }
    override PhysBody bodyB() @property { return physB_; }

    override void targetYaw(float yaw) @property { targetYaw_ = yaw; }
    override float targetYaw() @property const { return targetYaw_; }

    override float yawNow() @property const
    {
        Matrix4x4f m0, m1;
        NewtonBodyGetMatrix(steer_.newtonBody, m0.arrayof.ptr);
        NewtonBodyGetMatrix(master_.newtonBody, m1.arrayof.ptr);
        const vec3 up = m1.rotate(masterUpLocal_);
        const vec3 front = m1.rotate(Vector3f(0.0f, 0.0f, 1.0f));
        const vec3 steerFront = m0.rotate(Vector3f(0.0f, 0.0f, 1.0f));
        return calculateAngle(steerFront, front, up);
    }

    override void submit(float timestep, int threadIndex)
    {
        Matrix4x4f m0, m1;
        NewtonBodyGetMatrix(steer_.newtonBody, m0.arrayof.ptr);
        NewtonBodyGetMatrix(master_.newtonBody, m1.arrayof.ptr);

        const vec3 pivot0 = pivotSteerLocal_ * m0;
        const vec3 pivot1 = pivotMasterLocal_ * m1;

        const vec3 up = m1.rotate(masterUpLocal_);
        const vec3 front = m1.rotate(Vector3f(0.0f, 0.0f, 1.0f));
        const vec3 right = m1.rotate(Vector3f(1.0f, 0.0f, 0.0f));

        addLinearRow(pivot0, pivot1, front);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, up);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, right);
        setRowStiffness(1.0f);

        const vec3 armUp = m0.rotate(armUpLocal_);
        addAngularRow(calculateAngle(armUp, up, front), front);
        setRowStiffness(1.0f);
        addAngularRow(calculateAngle(armUp, up, right), right);
        setRowStiffness(1.0f);

        const vec3 steerFront = m0.rotate(Vector3f(0.0f, 0.0f, 1.0f));
        const float yaw = calculateAngle(steerFront, front, up);
        if (yaw < -limit_)
        {
            addAngularRow(yaw + limit_, up);
            setRowStiffness(1.0f);
        }
        else if (yaw > limit_)
        {
            addAngularRow(yaw - limit_, up);
            setRowStiffness(1.0f);
        }
        else
        {
            const float target = clamp(targetYaw_, -limit_, limit_);
            const float err = clamp(yaw - target, -errorCap_, errorCap_);
            addAngularRow(err, up);
            setRowStiffness(0.5f);
        }
    }
}

/* ------------------------------------------------------------------ *
 * Мир
 * ------------------------------------------------------------------ */

/// Мир Newton, поднятый до `PhysWorld`: роли вместо групп, контакты вместо
/// С++-колбэков, car-координаты вместо осей Newton.
final class NewtonPhysWorld : PhysWorld
{
    SoilWorld newton;
    ContactLog log_;

    /// Всё, что мир отдал за заезд. Формы, тела и шарниры числятся у мира
    /// (`dlib.Owner`) и живут до его гибели, а пул миры не уничтожает —
    /// без этих списков каждый заезд навечно оставлял бы миру свои объекты
    /// (`NewtonDestroyAllBodies` сносит только тела Newton, но не их
    /// обёртки, не формы и не шарниры).
    private NewtonConstraint[] joints_;
    private NewtonSensorBody[] bodies_;
    private NewtonCollisionShape[] shapes_;

    this()
    {
        ensureNewtonLoaded();
        newton = New!SoilWorld(cast(EventManager) null, cast(Owner) null);
    }

    override void useCallingThread() { newton.threadsCount = 0; }

    override void step(double dt)
    {
        log_.clear();
        newton.update(dt);
    }

    override const(ContactPair)[] contacts() const { return log_.pairs; }

    override PhysShape boxShape(const vec3 halfExtent)
    {
        shapes_ ~= New!NewtonBoxShape(halfExtent, newton);
        return New!Shape(shapes_[$ - 1]);
    }

    /// Цилиндр Newton создаётся с осью вдоль локальной X, а весь остальной
    /// код (и визуальный меш) считает ось цилиндра локальной Y. Поворот формы
    /// на +90° вокруг Z переводит X в Y, и меш сходится с коллизией.
    override PhysShape cylinderShape(float radius1, float radius2, float height)
    {
        auto shape = New!NewtonCylinderShape(radius1, radius2, height, newton);
        shape.setTransformation(
            rotationQuaternion(Vector3f(0, 0, 1), 0.5f * PI).toMatrix4x4);
        shapes_ ~= shape;
        return New!Shape(shape);
    }

    /// Статическая геометрия: нормали нужны дереву Newton, у нас их нет
    /// после преобразования осей, поэтому на грани считаем их заново.
    override PhysShape triangleMeshShape(const float[] vertices,
        const float[] normals, const uint[] indices)
    {
        shapes_ ~= New!TreeShape(newton, vertices, normals, indices);
        return New!Shape(shapes_[$ - 1]);
    }

    override PhysBody createFlatGround(float halfExtent, uint cells, float topZ)
    {
        shapes_ ~= New!FlatHeightfield(newton, halfExtent, cells);
        auto shape = New!Shape(shapes_[$ - 1]);
        // Верх земли — car-Z `topZ`, в Newton это плоскость Y; её левый
        // нижний угол уезжает по −Y (вперёд), то есть по +car-Y.
        auto b = createBody(BodyRole.ground, BodyMotion.staticBody, shape, 0.0f);
        b.setWorldPosition(vec3(-halfExtent, halfExtent, topZ));
        b.syncPose();
        return b;
    }

    override PhysBody createHeightfieldGround(const float[] heights, uint dims,
        float cell, const vec3 corner)
    {
        shapes_ ~= New!WindowHeightfield(newton, heights, dims, cell);
        auto shape = New!Shape(shapes_[$ - 1]);
        auto b = createBody(BodyRole.ground, BodyMotion.staticBody, shape, 0.0f);
        b.setWorldPosition(corner);
        b.syncPose();
        return b;
    }

    override PhysBody createBody(BodyRole role, BodyMotion motion,
        PhysShape shape, float mass)
    {
        NewtonRigidBodyType t;
        final switch (motion)
        {
            case BodyMotion.staticBody: t = NewtonRigidBodyType.Static; break;
            case BodyMotion.kinematicBody: t = NewtonRigidBodyType.Kinematic; break;
            case BodyMotion.dynamicBody: t = NewtonRigidBodyType.Dynamic; break;
        }
        auto s = cast(Shape) shape;
        enforce(s !is null, "форма выдана не этим бэкендом");
        if (s.inner !is null)
            shapes_ ~= s.inner;
        auto b = New!NewtonPhysBody(this, role, t, s.inner, mass, newton);
        bodies_ ~= b.body_;
        return b;
    }

    override void destroyBody(PhysBody body)
    {
        auto b = cast(NewtonPhysBody) body;
        if (b is null)
            return;
        // Тело снесено движком, обёртка и её форма — нами: иначе висят
        // указатели на высоты heightfield'а.
        NewtonDestroyBody(b.body_.newtonBody);
        untrack(bodies_, b.body_);
        newton.deleteOwnedObject(b.body_);
    }

    override void destroyShape(PhysShape shape)
    {
        auto s = cast(Shape) shape;
        if (s is null || s.inner is null)
            return;
        untrack(shapes_, s.inner);
        newton.deleteOwnedObject(s.inner);
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
        const int ga = groupOf(newton, rule.a);
        const int gb = groupOf(newton, rule.b);
        final switch (rule.response)
        {
            case ContactResponse.ignore:
                NewtonMaterialSetCollisionCallback(newton.newtonWorld, ga, gb,
                    &ignorePair, null);
                break;
            case ContactResponse.resolve:
                break;
            case ContactResponse.resolveAndReport:
                NewtonMaterialSetDefaultFriction(newton.newtonWorld, ga, gb,
                    rule.friction, rule.kineticFriction);
                NewtonMaterialSetDefaultElasticity(newton.newtonWorld, ga, gb,
                    rule.elasticity);
                NewtonMaterialSetCollisionCallback(newton.newtonWorld, ga, gb,
                    null, &reportPairCallback);
                break;
        }
    }

    /// Снять объект с учёта: `deleteOwnedObject` сравнивает указатели, и
    /// оставшийся в списке освобождённый адрес однажды совпал бы с новым
    /// живым объектом.
    private static void untrack(T)(ref T[] list, T obj)
    {
        foreach (i, x; list)
            if (x is obj)
            {
                list[i] = list[$ - 1];
                list.length = list.length - 1;
                return;
            }
    }

    override void clearScene()
    {
        // Порядок обязателен: шарнир ссылается на тела, а форма держит
        // указатель на буферы высот, поэтому сначала шарниры, потом тела,
        // потом формы.
        foreach (j; joints_)
            newton.deleteOwnedObject(j);
        NewtonDestroyAllBodies(newton.newtonWorld);
        foreach (b; bodies_)
            newton.deleteOwnedObject(b);
        foreach (sh; shapes_)
            newton.deleteOwnedObject(sh);
        joints_ = null;
        bodies_ = null;
        shapes_ = null;
        log_.clear();
    }

    override void dispose()
    {
        Delete(newton);
        newton = null;
        log_.dispose();
    }
}

PhysWorld createPhysWorld()
{
    return New!NewtonPhysWorld();
}
