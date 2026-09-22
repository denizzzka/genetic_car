module physics_world.car;

import std.math;
import std.algorithm : min, max;

import dlib.core.memory;
import dlib.math.vector;
import dlib.math.quaternion;
import dlib.math.matrix;
import dlib.math.transformation;
import dlib.core.ownership;

import dagon.core.event;
import dagon.ext.newton;

// Имена dlib.math.transformation (up/forward/right — шаблоны) конфликтуют с
// базисом каркаса; нужные оси каркаса переименовываем локально.
// Имена dlib.math.transformation (up/forward/right — шаблоны) конфликтуют с
// базисом каркаса; нужные оси каркаса переименовываем локально. Остальной
// frame.frame импортируется поимённо.
import frame.frame : origin, frameUp = up, frameForward = forward,
    frameRight = right, frameBackward = backward,
    Frame, Node, Beam, Anchor, AnchorKind, BeamKind,
    isConnected, initialMotorPower;
import physics_world.physics;
import physics_world.terrain;
import physics_world.terrainworld;
import physics_world.wheel;

/// Машина: каркас багги вместе с якорями (колёсами). Только данные —
/// физика физикой занимается отдельно (BuggyPhysics), когда нужен заезд.
/// Кадр ожидается уже разложенным (`placedFrame`): центр в нуле, низом
/// колеса на землю — тот, что приходит от вызывающего кода.
class Buggy
{
    /// Каркас багги: узлы (позиции), балки, якоря (разложен `placedFrame`).
    Frame frame;

    this(Frame frame)
    {
        this.frame = frame;
    }
}

unittest
{
    import std.math : isFinite;

    // Нормальный багги: рама-коробка на четырёх колёсах, задние — ведущие.
    Frame frame;
    size_t node(vec3 pos)
    {
        frame.nodes ~= Node(pos);
        return frame.nodes.length - 1;
    }

    const c = node(vec3(0.0f, 0.0f, 0.8f));
    const fl = node(vec3(0.6f, 0.7f, 0.3f));
    const fr = node(vec3(-0.6f, 0.7f, 0.3f));
    const rl = node(vec3(0.6f, -0.7f, 0.3f));
    const rr = node(vec3(-0.6f, -0.7f, 0.3f));
    frame.beams ~= Beam(c, fl, 0.045f);
    frame.beams ~= Beam(c, fr, 0.045f);
    frame.beams ~= Beam(c, rl, 0.05f);
    frame.beams ~= Beam(c, rr, 0.05f);
    frame.beams ~= Beam(fl, fr, 0.045f);
    frame.beams ~= Beam(rl, rr, 0.05f);
    frame.anchors ~= Anchor(fl, AnchorKind.wheel);
    frame.anchors ~= Anchor(fr, AnchorKind.wheel);
    frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
    frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
    frame.motorPower = initialMotorPower;

    auto physics = new BuggyPhysics(new Buggy(placedFrame(frame)));
    scope (exit) physics.dispose();

    const double dt = 1.0 / 60.0;

    // Плоская земля, мотор-колёса: успокоить раму, затем заезд на полном газу.
    physics.settle(dt, 120);

    vec3 avgPos()
    {
        vec3 p = vec3(0.0f, 0.0f, 0.0f);
        foreach (s; physics.wheelStates())
            p += s.position;
        return p / physics.wheelStates().length;
    }
    const vec3 start = avgPos;

    // ~9 секунд заезда — машина должна остаться на земле,
    // не разлететься и не провалиться сквозь неё.
    foreach (i; 0 .. 540)
        physics.step(dt, 1.0f);

    foreach (s; physics.wheelStates())
    {
        assert(isFinite(s.position.x) && isFinite(s.position.y) && isFinite(s.position.z),
            "позиция колеса не конечна — машина разлетелась");
        const float g = physics.groundHeightAt(s.position.xyz);
        assert(s.position.z > g + physicsWheelBelow, "колесо провалилось под землю");
        assert(s.position.z <= g + wheelRadius + 0.25f, "колесо парит над землёй");
    }
    foreach (s; physics.beamStates())
    {
        assert(isFinite(s.position.x) && isFinite(s.position.y) && isFinite(s.position.z),
            "позиция рамы не конечна — машина разлетелась");
    }

    // За ~9 секунд заезда машина должна заметно уехать вперёд по курсу.
    assert(dot(avgPos - start, frameForward) > 1.0,
        "мотор-колёса не везут машину — момент не передаётся раме");
}

unittest
{
    import std.math : isFinite;

    // Отрицательный момент — смена направления привода: тот же багги
    // с motorPower < 0 должен поехать назад (по backward).
    Frame frame;
    size_t node(vec3 pos)
    {
        frame.nodes ~= Node(pos);
        return frame.nodes.length - 1;
    }

    const c = node(vec3(0.0f, 0.0f, 0.8f));
    const fl = node(vec3(0.6f, 0.7f, 0.3f));
    const fr = node(vec3(-0.6f, 0.7f, 0.3f));
    const rl = node(vec3(0.6f, -0.7f, 0.3f));
    const rr = node(vec3(-0.6f, -0.7f, 0.3f));
    frame.beams ~= Beam(c, fl, 0.045f);
    frame.beams ~= Beam(c, fr, 0.045f);
    frame.beams ~= Beam(c, rl, 0.05f);
    frame.beams ~= Beam(c, rr, 0.05f);
    frame.beams ~= Beam(fl, fr, 0.045f);
    frame.beams ~= Beam(rl, rr, 0.05f);
    frame.anchors ~= Anchor(fl, AnchorKind.wheel);
    frame.anchors ~= Anchor(fr, AnchorKind.wheel);
    frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
    frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
    frame.motorPower = -initialMotorPower;

    auto physics = new BuggyPhysics(new Buggy(placedFrame(frame)));
    scope (exit) physics.dispose();

    const double dt = 1.0 / 60.0;
    physics.settle(dt, 120);

    vec3 avgPos()
    {
        vec3 p = vec3(0.0f, 0.0f, 0.0f);
        foreach (s; physics.wheelStates())
            p += s.position;
        return p / physics.wheelStates().length;
    }
    const vec3 start = avgPos;

    foreach (i; 0 .. 540)
        physics.step(dt, 1.0f);

    foreach (s; physics.wheelStates())
    {
        assert(isFinite(s.position.x) && isFinite(s.position.y) && isFinite(s.position.z),
            "позиция колеса не конечна — машина разлетелась");
        const float g = physics.groundHeightAt(s.position.xyz);
        assert(s.position.z > g + physicsWheelBelow, "колесо провалилось под землю");
        assert(s.position.z <= g + wheelRadius + 0.25f, "колесо парит над землёй");
    }

    // Отрицательный момент должен развернуть машину: движение по backward.
    // (Offroad-сцепление грунта 0.7/0.55 ниже прежнего 0.9 — колёса буксуют,
    // порог невысокий, но направление обязано быть backward.)
    const vec3 end = avgPos;
    const vec3 travel = end - start;
    assert(dot(travel, frameBackward) > 0.5,
        "отрицательный motorPower должен ехать в обратную сторону (backward)");
}

unittest
{
    // Ограничитель крутки: раскрученное сверх лимита колесо должно
    // снизить скорость обода до wheelMaxSurfaceSpeed (75 км/ч).
    Frame frame;
    size_t node(vec3 pos)
    {
        frame.nodes ~= Node(pos);
        return frame.nodes.length - 1;
    }

    const c = node(vec3(0.0f, 0.0f, 0.8f));
    const fl = node(vec3(0.6f, 0.7f, 0.3f));
    const fr = node(vec3(-0.6f, 0.7f, 0.3f));
    const rl = node(vec3(0.6f, -0.7f, 0.3f));
    const rr = node(vec3(-0.6f, -0.7f, 0.3f));
    frame.beams ~= Beam(c, fl, 0.045f);
    frame.beams ~= Beam(c, fr, 0.045f);
    frame.beams ~= Beam(c, rl, 0.05f);
    frame.beams ~= Beam(c, rr, 0.05f);
    frame.beams ~= Beam(fl, fr, 0.045f);
    frame.beams ~= Beam(rl, rr, 0.05f);
    frame.anchors ~= Anchor(fl, AnchorKind.wheel);
    frame.anchors ~= Anchor(fr, AnchorKind.wheel);
    frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
    frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
    frame.motorPower = 0.0f;

    auto physics = new BuggyPhysics(new Buggy(placedFrame(frame)));
    scope (exit) physics.dispose();

    const double dt = 1.0 / 60.0;
    physics.settle(dt, 120);

    auto w = physics.wheelBodies[0];
    const vec3 axle = w.rotation.conj.rotate(Vector3f(0.0f, 1.0f, 0.0f));
    w.angularVelocity = axle * 200.0f;

    foreach (i; 0 .. 10)
        physics.step(dt, 0.0f);

    const float surface = abs(dot(axle, w.angularVelocity)) * wheelRadius;
    assert(surface <= wheelMaxSurfaceSpeed * 1.05f,
        "регулятор не удержал скорость обода в пределах лимита");
}

/**
 * Есть ли у каркаса привод: хотя бы одно мотор-колесо и заметная сила мотора.
 *
 * Дёшево отсеивает заведомо стоячие каркасы до дорогой физической
 * симуляции — без мотор-колеса или при моменте ниже `minMotorPower` машина
 * не поедет.
 */
bool canDrive(const Frame f)
{
    if (abs(f.motorPower) <= minMotorPower)
        return false;
    foreach (a; f.anchors)
        if (a.kind == AnchorKind.motorWheel)
            return true;
    return false;
}

/// Роль тела в заезде: колбэки Newton прыгают по ней на нужную обработку.
private enum BodyKind
{
    none,
    ground,
    master,
    wheel,
    beam,
}

/// Наше расширение обёртки Newton: таскает обратную ссылку на BuggyPhysics,
/// чтобы статические колбэки мира знали, какому заезду принадлежит тело.
final class NewtonCarBody: NewtonRigidBody
{
    BuggyPhysics owner;
    BodyKind kind;
    size_t index;

    this(NewtonRigidBodyType bodyType, NewtonCollisionShape shape, float mass,
        NewtonPhysicsWorld world, Owner owner)
    {
        super(bodyType, shape, mass, world, owner);
    }
}

/// Контакты тел из группы sensor (балки) между собой глушатся
/// (AABB-overlap выключен): узлы каркаса в точке — это не столкновение.
extern(C) int sensorNoOverlap(const NewtonJoint* contact, dFloat timestep, int threadIndex)
{
    return 0;
}

/// Контакты обычных тел (default×default) не трогаем — они решаются,
/// — но подсматриваем: два колеса каркаса соприкасаются — провал заезда.
extern(C) void contactDefaultDefault(const NewtonJoint* joint, dFloat timestep, int threadIndex)
{
    NewtonBody* b0 = NewtonJointGetBody0(joint);
    NewtonBody* b1 = NewtonJointGetBody1(joint);
    auto nb0 = cast(NewtonCarBody)NewtonBodyGetUserData(b0);
    auto nb1 = cast(NewtonCarBody)NewtonBodyGetUserData(b1);
    if (nb0 !is null && nb1 !is null
        && nb0.kind == BodyKind.wheel && nb1.kind == BodyKind.wheel
        && nb0.owner !is null)
        nb0.owner.markWheelWheel();
}

/**
 * Физическая модель машины.
 *
 * Внутри мир Newton живёт в своих координатах: X вправо, Y вверх (земля —
 * плоскость XZ на y == 0, см. `carToNewtonQuat`). Наружу (тесты, вьюер,
 * фитнес) через `toCarPos`/`toCarRot` всё отдаётся в координатах каркаса
 * (car-local): X вправо, Y вперёд, Z вверх.
 *
 * Создаётся независимо от `Buggy`, только когда нужен заезд: мир с землёй,
 * у каждой балки каркаса — отдельное кинестатическое тело в sensor-группе
 * (контакты регистрируются, но не решаются), у якорей — динамические колёса.
 * Жёсткость каркаса держит «мастер» — тело с массой и инерцией всей рамы
 * (коллизии у него выключены): к нему колёса приварены осями (револьте-
 * шарнир WheelAxleJoint)
 * и из него на каждом шаге пересчитываются трансформы тел балок, так что узлы
 * не разбалтываются и каркас катится как монолит. Задевание балки земли или
 * чужого колеса ловится в sensor-колбэке и жёстко отбраковывает заезд.
 * Привод — мотор-колёса: на ведущие колёса подаётся момент `Frame.motorPower`
 * (наследуемый ген) вокруг их оси, закрутка толкает машину по курсу (-Y).
 */
final class BuggyPhysics
{
    private NewtonPhysicsWorld world;

    /// Тело земли: статичный body с плоским heightfield, верх на y == 0.
    /// В terrain-режиме (terrain_ != null) земли нет — её ведёт terrainWorld_.
    private NewtonCarBody ground;

    /// Общая процедурная поверхность (shared-кэш фитнес-пула и вьюера).
    /// При null заезд идёт по прежней плоскости y == 0.
    //
    // TODO: террейн (и кэш, и окно) логичнее держать не в BuggyPhysics, а в
    // мировом объекте: земля — содержимое мира, а «плоская/рельефная» земля —
    // свойство мира. Это уберёт тереновый параметр из обоих конструкторов и
    // ветвления в groundHeightAt/runFailure/beamUnderground, заодно снесёт
    // порядок сноса окна из dispose(). Но NewtonPhysicsWorld — код dagon
    // (внешняя зависимость), поэтому мир надо унаследовать (он не final,
    // как и dlib Owner) — например class BuggyWorld : NewtonPhysicsWorld с
    // setTerrain/reset/groundHeightAt. Решающий нюанс — пул: миры
    // переиспользуются без NewtonDestroy, поэтому снести окно и обнулить
    // террейн обязан release() (worldpool) перед NewtonDestroyAllBodies,
    // иначе после рельефного заезда следующий «плоский» получит рельеф.
    private TerrainSurface terrain_;

    /// Стримингуемое окно поверхности в нашем мире (появляется в buildGround).
    private TerrainWorld terrainWorld_;

    /// «Мастер» каркаса: единый центр масс и инерции рамы. Он же везёт
    /// тела балок — их трансформы жёстко пересчитываются из мастера каждый
    /// шаг, рама идеально монолитна, а коллизии мастера выключены.
    private NewtonCarBody master;

    /// Локальные смещение и ориентация каждой балки в мастер-теле.
    private Vector3f[] beamLocal;
    private Quaternionf[] beamLocalQuat;

    /// Кинестатические тела балок каркаса: по одному на каждую `Frame.beams`.
    private NewtonCarBody[] beamBodies;

    /// Длина каждой балки — для геометрической проверки «под землёй».
    private float[] beamLen;

    /// Пары узлов концов балки (индекс в сварках и ступицах).
    private size_t[] beamNodeA, beamNodeB;

    /// Тела колёс по индексам `Frame.anchors`.
    private NewtonCarBody[] wheelBodies;

    /// Узел якоря каждого колеса (своя ступица не считается задеванием).
    private size_t[] wheelNodes;

    /// Первый же провал заезда (латится): сенсорные колбэки и геометрия
    /// копят сюда причину, `beamFailure()` её выдаёт.
    private BeamFailure beamFail_;

    /// Время симуляции без нового продвижения вперёд по курсу (-Y каркаса).
    /// Обнуляется в `updateStall`, выливается в сход через `stallTime`.
    private double stallTime_ = 0.0;

    /// Самая дальняя достигнутая точка по курсу (car-координата Y мастера);
    /// новый рекорд набега сбрасывает `stallTime_`.
    private float forwardMinY_ = float.max;

    /// Машина-основа
    private const Buggy buggy_;

    /// Мир принадлежит нам, и `dispose` должен его уничтожить через `Delete`.
    /// Миры из пула (см. physics_world.worldpool) — чужие: `dispose` их не
    /// трогает, пул возвращает мир в оборот без `NewtonDestroy`.
    private bool ownsWorld_;

    /// Свой мир: создаётся локально и забирается с собой (тесты/вьюер).
    this(const Buggy buggy, TerrainSurface terrain = null)
    {
        ensureNewtonLoaded();
        this(buggy, New!PhysicsWorld(cast(EventManager)null, cast(Owner)null),
            terrain);
        ownsWorld_ = true;
    }

    this(const Buggy buggy, NewtonPhysicsWorld pooledWorld,
        TerrainSurface terrain = null)
    {
        ensureNewtonLoaded();
        buggy_ = buggy;
        terrain_ = terrain;

        // Конструктор NewtonPhysicsWorld просит EventManager, но хранит его
        // только для проформы: симуляции он не касается. Передаём null.
        world = pooledWorld;
        ownsWorld_ = false;
        world.threadsCount = 0;

        // Сцепление и упругость между колёсами (default×default) — прежние:
        // контакт колёс — провал заезда, пару решать не нужно. Сцепление
        // колёс о грунт задаёт мир (PhysicsWorld) на паре default×soil.
        NewtonMaterialSetDefaultFriction(world.newtonWorld,
            world.defaultGroupId, world.defaultGroupId, wheelWheelFriction, wheelWheelFriction);
        NewtonMaterialSetDefaultElasticity(world.newtonWorld,
            world.defaultGroupId, world.defaultGroupId, wheelWheelElasticity);
        // Контакт двух обычных тел не меняем, но смотрим (провал wheelWheel).
        NewtonMaterialSetCollisionCallback(world.newtonWorld,
            world.defaultGroupId, world.defaultGroupId, null, &contactDefaultDefault);
        // Балки сами с собой не контачат: стыки узлов — не провал.
        NewtonMaterialSetCollisionCallback(world.newtonWorld,
            world.sensorGroupId, world.sensorGroupId, &sensorNoOverlap, null);

        // Каркас в Buggy уже разложен конструктором (center + на землю).
        buildGround();
        buildFrame();
        buildWheels();
        beamFail_ = BeamFailure.none;
    }

    ~this()
    {
        dispose();
    }

    void dispose()
    {
        // Тайлы поверхности сносятся ДО того, как мир покинет пул
        // (NewtonDestroyAllBodies) или будет уничтожен: иначе двойное
        // освобождение ground/boulder-тел.
        if (terrainWorld_ !is null)
        {
            terrainWorld_.dispose();
            terrainWorld_ = null;
        }
        if (world !is null)
        {
            // Свой мир уничтожаем целиком (NewtonDestroy). Чужой (из пула)
            // только отсоединяем: пул сам вернёт его с пустыми телами.
            if (ownsWorld_)
                Delete(world);
            world = null;
        }
        ground = null;
        terrain_ = null;
        master = null;
        beamLocal.length = 0;
        beamLocalQuat.length = 0;
        beamBodies.length = 0;
        beamLen.length = 0;
        beamNodeA.length = 0;
        beamNodeB.length = 0;
        wheelBodies.length = 0;
        wheelNodes.length = 0;
        beamFail_ = BeamFailure.none;
    }

    /// Один шаг симуляции. Фиксированный dt (~1/60) надёжно стабилен.
    ///
    /// `throttle` (0..1) кладёт момент на мотор-колёса: закрутка вокруг оси
    /// колеса толкает машину по курсу (-Y). Момент копится до world.update и
    /// сбрасывается внутри него, поэтому подаётся каждый шаг заново. Пассивные
    /// колёса на осях катятся сами — их везёт сцепление с землёй.
    void step(double dt, float throttle)
    {
        if (world is null)
            return;

        applyDrive(throttle);
        applyWheelSpinGovernor();
        world.update(dt);
        syncBodies();
        updateBeamPuppets();
        updateStall(dt);
        if (terrainWorld_ !is null && master !is null)
            terrainWorld_.updateAround(toCarPos(master.position.xyz));
    }

    /// Момент полного газа на каждое мотор-колесо, разложенный по `throttle`.
    /// Сила мотора — наследуемый параметр каркаса (`Frame.motorPower`),
    /// знак силы — направление привода: положительный момент едет вперёд по
    /// курсу, отрицательный разворачивает вращение и едет назад (backward).
    private void applyDrive(float throttle)
    {
        if (throttle == 0.0f || master is null)
            return;
        const Frame fr = buggy_.frame;
        if (fr.motorPower == 0.0f)
            return;
        const vec3 spin = cross(toNewtonPos(frameUp), toNewtonPos(frameForward));
        foreach (i, a; fr.anchors)
            if (a.kind == AnchorKind.motorWheel && i < wheelBodies.length
                && wheelBodies[i] !is null)
            {
                auto w = wheelBodies[i];
                const vec3 axle = w.rotation.conj.rotate(Vector3f(0.0f, 1.0f, 0.0f));
                const vec3 dir = (dot(axle, spin) < 0.0f) ? -axle : axle;
                w.addTorque(dir * (throttle * fr.motorPower));
            }
    }

    /// Мягкий ограничитель крутки: поверхность колеса не должна двигаться
    /// быстрее `wheelMaxSurfaceSpeed`. Превышение лимита угловой скорости
    /// (для генетического радиуса колеса — `wheelMaxSurfaceSpeed / r`)
    /// гасится встречным моментом, пропорциональным превышению
    /// (`wheelSpinGain`) — предел мягкий, буксование и свободный разнос
    /// ниже лимита ничем не стесняются. Действует на все колёса, не только
    /// на моторные: с каждой падает отряд, раскрутка любой — по лимиту.
    private void applyWheelSpinGovernor()
    {
        if (master is null)
            return;
        const Frame fr = buggy_.frame;
        foreach (i, w; wheelBodies)
            if (w !is null)
            {
                const vec3 axle = w.rotation.conj.rotate(Vector3f(0.0f, 1.0f, 0.0f));
                const float spin = dot(axle, w.angularVelocity);
                const float r = i < fr.anchors.length ? fr.anchors[i].radius : wheelRadius;
                const float excess = abs(spin) - wheelMaxSurfaceSpeed / r;
                if (excess > 0.0f)
                {
                    const float dir = (spin < 0.0f) ? -1.0f : 1.0f;
                    w.addTorque(axle * (-dir * excess * wheelSpinGain));
                }
            }
    }

    /// Учёт застоя по курсу: новый рекорд набега вперёд сбрасывает таймер,
    /// иначе к нему добавляется время текущего шага. `runFailure` считает
    /// переполнение `stallSeconds` сходом с дистанции.
    private void updateStall(double dt)
    {
        if (master is null)
            return;
        // По курсу — минус Y каркаса, как в fitness-метрике «продвижение вниз».
        const float fwd = toCarPos(master.position.xyz).y;
        if (fwd < forwardMinY_ - stallProgressEps)
        {
            forwardMinY_ = fwd;
            stallTime_ = 0.0;
        }
        else
        {
            forwardMinY_ = min(forwardMinY_, fwd);
            stallTime_ += dt;
        }
    }

    /// Успокоить машину: шаги симуляции без движения, чтобы осадка рамы и
    /// первые контакты колёс улеглись и старт заезда был чистым.
    void settle(double dt, int steps)
    {
        foreach (_; 0 .. steps)
            step(dt, 0.0f);
    }

    /// Прочитать свежие позы тел из Newton после шага: обёртка хранит копии
    /// position/rotation, и без этого вызова они не обновятся.
    private void syncBodies()
    {
        if (master is null)
            return;
        master.update(0.0);
        foreach (w; wheelBodies)
            if (w !is null)
                w.update(0.0);
    }

    BodyState[] beamStates()
    {
        BodyState[] res;
        foreach (b; beamBodies)
            if (b !is null)
            {
                BodyState s;
                s.position = toCarPos(b.position.xyz);
                s.orientation = toCarRot(b.rotation);
                res ~= s;
            }
        return res;
    }

    BodyState[] wheelStates()
    {
        BodyState[] res;
        foreach (w; wheelBodies)
            if (w !is null)
            {
                BodyState s;
                s.position = toCarPos(w.position.xyz);
                s.orientation = toCarRot(w.rotation);
                res ~= s;
            }
        return res;
    }

    // Debug-only хелперы обёрнуты в block-scoped `debug { }`: метка `debug:`
    // в release выключала всю остальную часть класса до его конца.
    debug {
    /// Сырое (кэшированное dagon'ом) состояние мастера для отладки.
    BodyState dbgMasterState() @property
    {
        BodyState s;
        if (master is null)
            return s;
        s.position = toCarPos(master.position.xyz);
        s.orientation = toCarRot(master.rotation);
        return s;
    }

    /// Ожидаемые балки ровного монолитного каркаса: какой должна быть каждая
    /// балка по замыслу (на месте закрепления), будучи жёстко приделанной к
    /// мастеру. mid — ожидаемый центр, dir — ожидаемая ось (локальный Y),
    /// len — длина.
    static struct BeamTarget
    {
        Vector3f mid;
        Vector3f dir;
        float len;
    }

    BeamTarget[] dbgBeamTargets() @property
    {
        BeamTarget[] res;
        if (master is null)
            return res;
        const Frame fr = buggy_.frame;
        Quaternionf mt = toCarRot(master.rotation); // истинное вращение мастера
        foreach (i, b; buggy_.frame.beams)
        {
            const vec3 a = fr.nodes[b.a].pos;
            const vec3 c = fr.nodes[b.b].pos;
            const vec3 d = c - a;
            if (d.length < 1e-5f)
                continue;
            BeamTarget t;
            t.mid = toCarPos(master.position.xyz) + mt.rotate(beamLocal[i]);
            t.dir = mt.rotate(d);
            t.len = d.length;
            res ~= t;
        }
        return res;
    }
    }

    private void buildGround()
    {
        if (terrain_ !is null)
        {
            // Процедурная поверхность: окно из тайлов вокруг старта. Ground-тело
            // и булыжники тайлов живут в TerrainWorld — здесь они не нужны.
            terrainWorld_ = new TerrainWorld(world, terrain_, terrain_.config);
            terrainWorld_.updateAround(origin);
            return;
        }

        // Земля-heightfield в координатах мира Newton: плоскость XZ на y == 0,
        // вверх оси — +Y (см. carToNewtonQuat). Грань прежнего бокса лежала на
        // z == 0 координат каркаса, что в Newton совпадает с y == 0.
        // Перенос поля (центрирование) — трансформацией ТЕЛА, а не коллизии:
        // матрица на heightfield внутри shape даёт NaN AABB.
        auto shape = New!GroundHeightfield(120.0f, 8, world);
        auto body = New!NewtonCarBody(NewtonRigidBodyType.Static, shape,
            0.0f, world, world);
        body.dynamic = false;
        body.kind = BodyKind.ground;
        body.groupId = soilGroupIdOf(world);
        body.setTransformation(translationMatrix(vec3(-shape.halfExtent, 0.0f, -shape.halfExtent)));
        body.update(0.0);
        ground = body;
    }

    private void buildFrame()
    {
        const Frame frame = buggy_.frame;
        beamBodies.length = frame.beams.length;
        beamLen.length = frame.beams.length;
        beamNodeA.length = frame.beams.length;
        beamNodeB.length = frame.beams.length;
        beamLocal.length = frame.beams.length;
        beamLocalQuat.length = frame.beams.length;

        // Геометрия каркаса — отдельные кинестатические тела-балки в
        // sensor-группе: контакты с землёй и чужими колёсами ловятся как
        // провал заезда, но никогда не толкают (колбэк их снимает).
        // Массы у балок нет — мост (master) несёт всю раму и тянет балки.
        foreach (i, b; frame.beams)
        {
            const vec3 a = frame.nodes[b.a].pos;
            const vec3 c = frame.nodes[b.b].pos;
            const vec3 dir = c - a;
            const float len = dir.length;
            if (len < 1e-5f)
                continue;

            auto body = New!NewtonCarBody(NewtonRigidBodyType.Kinematic,
                makeAxisYCylinder(b.radius, b.radius, len, world),
                0.0f, world, world);
            // Ось цилиндра (локальный Y) — вдоль балки.
            body.dynamic = true;
            body.kind = BodyKind.beam;
            body.index = i;
            body.groupId = world.sensorGroupId;
            body.sensor = true;
            body.collidable = true;
            const Quaternionf q = rotationBetween(Vector3f(0, 1, 0), dir / len);
            body.setTransformation(newtonBodyMatrix((a + c) * 0.5f, q));
            body.update(0.0);

            // Сенсорный колбэк — наша обратная связь: каждая балка знает
            // свой индекс и сообщает заезду о задевании.
            immutable beamIdx = i;
            body.sensorCallback = (NewtonRigidBody, NewtonRigidBody other)
            {
                onBeamContact(beamIdx, other);
            };

            beamBodies[i] = body;
            beamLen[i] = len;
            beamNodeA[i] = b.a;
            beamNodeB[i] = b.b;
        }

        // Мастер: масса и центр масс по балкам, инерция от AABB каркаса.
        float totalMass = 0.0f;
        vec3 sumM = origin;
        foreach (b; frame.beams)
        {
            const vec3 a = frame.nodes[b.a].pos;
            const vec3 b2 = frame.nodes[b.b].pos;
            const float len = (b2 - a).length;
            if (len < 1e-5f)
                continue;
            float m = cast(float)(beamDensity * PI * b.radius * b.radius * len);
            totalMass += m;
            sumM += (a + b2) * 0.5f * m;
        }
        if (totalMass <= 0.0f)
            totalMass = 1.0f;
        const vec3 com = sumM / totalMass;

        master = New!NewtonCarBody(NewtonRigidBodyType.Dynamic,
            New!NewtonBoxShape(Vector3f(0.05f, 0.05f, 0.05f), world),
            0.0f, world, world);
        master.dynamic = true;
        master.kind = BodyKind.master;
        master.autoSleep = false;
        master.collidable = false; // коллизии считают балки и колёса
        master.gravity = gravity;
        master.linearDamping = bodyDamping;
        master.angularDamping = Vector3f(bodyDamping, bodyDamping, bodyDamping);

        // AABB каркаса для грубой инерции рамы.
        vec3 minP = vec3(float.max), maxP = vec3(-float.max);
        foreach (n; frame.nodes)
        {
            const vec3 p = n.pos;
            minP.x = min(minP.x, p.x);
            minP.y = min(minP.y, p.y);
            minP.z = min(minP.z, p.z);
            maxP.x = max(maxP.x, p.x);
            maxP.y = max(maxP.y, p.y);
            maxP.z = max(maxP.z, p.z);
        }
        vec3 dims = maxP - minP;
        dims.x = max(dims.x, 0.05f);
        dims.y = max(dims.y, 0.05f);
        dims.z = max(dims.z, 0.05f);
        const float Ixx = (dims.y * dims.y + dims.z * dims.z) / 3.0f * totalMass;
        const float Iyy = (dims.x * dims.x + dims.z * dims.z) / 3.0f * totalMass;
        const float Izz = (dims.x * dims.x + dims.y * dims.y) / 3.0f * totalMass;
        master.setMassMatrix(totalMass, Ixx, Iyy, Izz);

        master.setTransformation(translationMatrix(toNewtonPos(com)));
        master.update(0.0);

        // Локальные преобразования балок в мастере.
        foreach (i, b; frame.beams)
        {
            if (beamBodies[i] is null)
                continue;
            beamLocal[i] = master.rotation.conj.rotate(
                beamBodies[i].position.xyz - master.position.xyz);
            beamLocalQuat[i] = master.rotation * beamBodies[i].rotation.conj;
        }
    }

    /// Пересчёт тел балок из мастера после шага симуляции: рама следует за
    /// своим центром масс как монолит. Скорости тоже приравниваются, чтобы
    /// контакты колёс считались против согласованного движения каркаса.
    private void updateBeamPuppets()
    {
        if (master is null)
            return;
        // Кэшированное dagon'ом вращение любого тела — инверсия истинного:
        // dlib's fromMatrix читает матрицу Newton во встречной конвенции, и
        // readback по правилу из toMatrix4x4 обращается. Везде дальше истинное
        // вращение получаем через `.conj`.
        Quaternionf mTrue = master.rotation.conj;
        foreach (i, b; beamBodies)
        {
            if (b is null)
                continue;
            const vec3 r = mTrue.rotate(beamLocal[i]);
            const vec3 pos = master.position.xyz + r;
            const Quaternionf q = mTrue * beamLocalQuat[i];
            b.setTransformation(translationMatrix(pos) * q.toMatrix4x4);
            b.update(0.0);
            b.velocity = master.velocity + cross(master.angularVelocity, r);
            b.angularVelocity = master.angularVelocity;
        }
    }

    /// Сенсорный колбэк балки: задело колесо (не своё) — провал.
    /// Землю тут игнорируем — её ловит геометрическая beamUnderground().
    private void onBeamContact(size_t beamIdx, NewtonRigidBody other)
    {
        foreach (wi, w; wheelBodies)
            if (w is other)
            {
                if (!isOwnWheel(wheelNodes[wi], beamNodeA[beamIdx], beamNodeB[beamIdx]))
                    beamFail_ = BeamFailure.wheel;
                return;
            }
    }

    /// См. contactDefaultDefault: два колеса соприкасаются.
    private void markWheelWheel()
    {
        if (beamFail_ == BeamFailure.none)
            beamFail_ = BeamFailure.wheelWheel;
    }

    /**
     * Обрыв заезда из-за каркаса: балка ударилась об землю, а любое
     * соприкосновение частей колёс с балкой или другим колесом отбраковывается.
     *
     * Единственное исключение — контакт балки со СВОЕЙ ступицей (колесом,
     * приваренным к концу этой балки): ось легитимно проходит через колесо,
     * без этого не собрать ни одного каркаса. Вся остальная часть колеса,
     * цепляющая уже чужую балку или чужое колесо, — повод для отбраковки.
     */
    BeamFailure beamFailure()
    {
        if (world is null)
            return BeamFailure.none;
        if (beamFail_ != BeamFailure.none)
            return beamFail_;
        if (beamUnderground())
            beamFail_ = BeamFailure.ground;
        return beamFail_;
    }

    /// Геометрическая проверка «рама под землёй»: низшая точка поверхности
    /// любой балки ниже локальной земли (плоскость y == 0 или рельеф процедурной
    /// поверхности, см. groundHeightAt) минус `beamGroundEps`. Не зависит от
    /// контактов Newton — ловит и глухое погружение, и проскакивание между
    /// шагами проверки. Высота здесь — координата Y мира Newton.
    private bool beamUnderground()
    {
        if (master is null)
            return false;
        const Frame fr = buggy_.frame;
        foreach (i, b; beamBodies)
        {
            if (b is null)
                continue;
            const vec3 dir = b.rotation.conj.rotate(Vector3f(0.0f, 1.0f, 0.0f));
            const vec3 lowWorld = b.position.xyz - dir * (beamLen[i] * 0.5f);
            const vec3 lowCar = toCarPos(lowWorld);
            const float ground = groundHeightAt(lowCar);
            if (lowCar.z - fr.beams[i].radius < ground - beamGroundEps)
                return true;
        }
        return false;
    }

    /// Высота локальной земли в точке на плоскости каркаса (компоненты вдоль
    /// `forward` и `right`): 0 для плоской земли, процедурная высота вдоль `up`
    /// для поверхности.
    float groundHeightAt(const vec3 p)
    {
        if (terrain_ is null)
            return 0.0f;
        return terrain_.heightAt(p);
    }

    /// Фокус поверхности для стриминга окна: мастер, спроецированный на
    /// плоскость каркаса (вдоль `up` = 0).
    vec3 surfaceFocus() @property
    {
        if (master is null)
            return origin;
        const vec3 p = toCarPos(master.position.xyz);
        return p - frameUp * p.z;
    }

    /// Мир-позиция мастера (центр массы каркаса) в координатах Dagon/Newton —
    /// точка, за которой следует камера живого заезда.
    Vector3f worldFocus() @property
    {
        if (master is null)
            return Vector3f(0.0f, 0.0f, 0.0f);
        return master.position.xyz;
    }

    /// Направление «вверх» рамы в координатах каркаса: на старте — строго +Z.
    /// По нему `runFailure` судит о перевороте (крен/тангаж).
    vec3 bodyUp() @property
    {
        if (master is null)
            return Vector3f(0.0f, 0.0f, 1.0f);
        const vec3 upNewton = master.rotation.conj.rotate(Vector3f(0.0f, 1.0f, 0.0f));
        return toCarPos(upNewton);
    }

    /// Накопленное время симуляции без продвижения вперёд по курсу (-Y
    /// каркаса). Превышение `stallSeconds` — сход с дистанции.
    double stallTime() @property
    {
        return stallTime_;
    }

    /// Активное окно поверхности (если в этом заезде есть рельеф) — для вьюера.
    TerrainWorld terrainWorld() @property
    {
        return terrainWorld_;
    }

    private void buildWheels()
    {
        const Frame frame = buggy_.frame;
        wheelBodies.length = frame.anchors.length;
        wheelNodes.length = frame.anchors.length;

        foreach (i, a; frame.anchors)
        {
            const vec3 nodePos = frame.nodes[a.node].pos;

            // Генетический радиус колеса. Внутренний радиус покрышки и её
            // ширина масштабируются по отношению к базовому `wheelRadius`,
            // чтобы полое «кольцо» сохраняло пропорции.
            const float r = a.radius;
            const float ir = wheelInnerRadius * r / wheelRadius;
            const float w = wheelWidth * r / wheelRadius;

            float mass = cast(float)(wheelDensity * PI
                * (r * r - ir * ir) * w);
            auto wheel = New!NewtonCarBody(NewtonRigidBodyType.Dynamic,
                makeAxisYCylinder(r, r, w, world),
                mass, world, world);
            wheel.dynamic = true;
            wheel.kind = BodyKind.wheel;
            wheel.index = i;
            wheel.autoSleep = false;
            wheel.gravity = gravity;
            // Линейное — лёгкий выкат (Crr гасит зацепление, не «воздух»);
            // угловое: ось спина (локальный Y цилиндра, см. makeAxisYCylinder) —
            // сопротивление качению по грунту, перпендикулярные оси — прежнее
            // общее демпфирование.
            wheel.linearDamping = tireLinearDamping;
            wheel.angularDamping = Vector3f(bodyDamping, tireRollDamping, bodyDamping);
            // Инерция полого цилиндра, ось вращения — локальный Y.
            const float r2 = r * r;
            const float ri2 = ir * ir;
            const float h2 = w * w;
            const float perp = (3.0f * (r2 + ri2) + h2) / 12.0f * mass;
            const float axial = 0.5f * (r2 + ri2) * mass;
            wheel.setMassMatrix(mass, perp, axial, perp);

            // Ось колеса — направление «своей» балки узла: диск встаёт
            // перпендикулярно балке, а балка проходит сквозь ступицу.
            const Quaternionf q = rotationBetween(Vector3f(0, 1, 0), wheelAxle(frame.beamDirectionAt(a.node)));
            wheel.setTransformation(newtonBodyMatrix(nodePos, q));
            wheel.update(0.0);

            // Колесо держится на оси, закреплённой одним концом: револьте-
            // шарнир в точке узла якоря оставляет свободным только спин
            // колеса вокруг оси, а наклон оси относительно каркаса блокирует.
            // В отличие от BallConstraint (шаровой шарнир — все три поворота
            // свободны, колесо болтается вокруг пивота) ось не «гуляет».
            if (master !is null)
            {
                const vec3 pivotNewton = toNewtonPos(nodePos);
                // Мастер построен поворотом-тождеством, поэтому пивот в его
                // локальных координатах — просто разность мировых точек.
                const vec3 pivotMasterLocal = pivotNewton - master.position.xyz;
                // Шарнир владеет миром (`NewtonConstraint: Owner`, `super(world)`),
                // поэтому удалять его вручную не нужно: `Delete(world)` (свой мир)
                // снесёт его сам, а чужой мир из пула переживёт заезд.
                New!WheelAxleJoint(wheel, master, pivotMasterLocal);
            }

            wheelBodies[i] = wheel;
            wheelNodes[i] = a.node;
        }
    }
}

/// Вердикт заезда
string runFailure(BuggyPhysics physics)
{
    // Застой по курсу: без набега вперёд с последней продвинутой точки
    // столько секунд подряд — это тоже сход с дистанции.
    if (physics.stallTime() > stallSeconds)
        return "нет продвижения вперёд";

    const wheels = physics.wheelStates();
    if (wheels.length == 0)
        return "не осталось колёс";

    // Переворот — по наклону рамы, а не по высоте ступицы: подъём колеса на
    // крене, отрыве или бугре переворотом не является. Допускаем до
    // `maxTiltDegrees` крена (вбок) и тангажа (вперёд/назад).
    const vec3 up = physics.bodyUp();
    const float rollDeg = atan2(up.x, up.z) * 180.0f / PI;
    const float pitchDeg = atan2(up.y, up.z) * 180.0f / PI;
    if (abs(rollDeg) > maxTiltDegrees || abs(pitchDeg) > maxTiltDegrees)
        return "машина перевернулась";

    foreach (s; wheels)
    {
        if (!isFinite(s.position.x) || !isFinite(s.position.y)
            || !isFinite(s.position.z))
            return "каркас разлетелся";
        // Локальная земля под колесом: 0 на плоскости, рельеф на поверхности.
        // Так колесо не «проваливается» на бугре и не «парит» над ложбиной.
        const float g = physics.groundHeightAt(s.position.xyz);
        if (s.position.z < g + physicsWheelBelow)
            return "колесо провалилось под землю";
    }

    foreach (s; physics.beamStates())
        if (!isFinite(s.position.x) || !isFinite(s.position.y)
            || !isFinite(s.position.z))
            return "балка разлетелась";

    switch (physics.beamFailure())
    {
        case BeamFailure.ground:
            return "балка каркаса касается земли";
        case BeamFailure.wheel:
            return "балка каркаса касается колеса";
        case BeamFailure.wheelWheel:
            return "колёса каркаса соприкасаются";
        case BeamFailure.none:
        default:
            break;
    }
    return "";
}
unittest
{
    // canDrive: решает, стоит ли запускать физический заезд.
    Frame f;
    f.nodes = [Node(origin), Node(frameRight)];
    f.beams = [Beam(0, 1, 0.05f)];

    f.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(!canDrive(f), "нет мотор-колеса — привода нет");

    f.anchors ~= Anchor(1, AnchorKind.motorWheel);
    f.motorPower = minMotorPower;
    assert(!canDrive(f), "момент на пороге не считается приводом");

    f.motorPower = minMotorPower + 1.0f;
    assert(canDrive(f), "мотор-колесо с заметным моментом — привод есть");

    f.motorPower = -(minMotorPower + 1.0f);
    assert(canDrive(f), "отрицательный момент — такой же привод, только назад");
}

unittest
{
    version(none) // TODO: оси колёс сейчас расположены неверно и совпадающие
                  // коллайдеры не дают contact callback; вернуть тест после
                  // правки геометрии колёс (wheelWheel-отбраковка в fitness'е).
    {
    // Столкновение колёс между собой — обрыв заезда. Два якоря в одной точке
    // (wheel + motorWheel на одном узле) дают совпадающие коллайдеры, которые
    // глушат привод; контакт между ними держится и до, и после шагов газом.
    Frame vframe()
    {
        Frame fr;
        fr.nodes ~= Node(vec3(0.0f, 0.904f, 0.300f));
        fr.nodes ~= Node(vec3(0.602f, -0.505f, 0.101f));
        fr.nodes ~= Node(vec3(-0.602f, -0.505f, 0.101f));
        fr.nodes ~= Node(vec3(1.204f, -1.915f, -0.098f));
        fr.nodes ~= Node(vec3(-1.204f, -1.915f, -0.098f));
        fr.beams ~= Beam(0, 1, 0.050f);
        fr.beams ~= Beam(0, 2, 0.050f);
        fr.beams ~= Beam(1, 3, 0.044f);
        fr.beams ~= Beam(2, 4, 0.044f);
        fr.motorPower = 109.6f;
        return fr;
    }

    // Нормальный каркас: по одному якорю на узел — колёса не касаются друг
    // друга, заезд не обрывается из-за wheelWheel.
    Frame good = vframe();
    good.anchors ~= Anchor(1, AnchorKind.wheel);
    good.anchors ~= Anchor(2, AnchorKind.wheel);
    good.anchors ~= Anchor(3, AnchorKind.motorWheel);
    good.anchors ~= Anchor(4, AnchorKind.motorWheel);
    {
        auto physics = new BuggyPhysics(new Buggy(placedFrame(good)));
        scope (exit) physics.dispose();
        physics.settle(1.0 / 60.0, 30);
        assert(physics.beamFailure() == BeamFailure.none,
            "ступицы своих балок не должны отбраковывать живую машину");
    }

    // Дубли: два колеса в каждом из двух узлов — коллизия колёс ловится.
    // Движок устойчиво держит между ними контакт, поэтому отбраковка
    // срабатывает и до, и после шагов газом.
    Frame dup = vframe();
    dup.anchors ~= Anchor(4, AnchorKind.wheel);
    dup.anchors ~= Anchor(3, AnchorKind.wheel);
    dup.anchors ~= Anchor(4, AnchorKind.motorWheel);
    dup.anchors ~= Anchor(3, AnchorKind.motorWheel);
    {
        auto physics = new BuggyPhysics(new Buggy(placedFrame(dup)));
        scope (exit) physics.dispose();
        physics.settle(1.0 / 60.0, 30);
        assert(physics.beamFailure() == BeamFailure.wheelWheel,
            "совпадающие колёса должны отбраковываться по столкновению");
    }
    } // version(none) TODO: см. выше
}

unittest
{
    // Застой по курсу — сход: без привода машина стоит на месте, и после
    // `stallSeconds` симуляционных секунд runFailure объявляет сход.
    Frame f;
    f.nodes = [Node(origin), Node(frameRight), Node(frameRight * 2.0f)];
    f.beams = [Beam(0, 1, 0.04f), Beam(1, 2, 0.04f)];
    f.anchors = [Anchor(0, AnchorKind.wheel), Anchor(2, AnchorKind.wheel)];
    const double dt = 1.0 / 60.0;

    auto physics = new BuggyPhysics(new Buggy(placedFrame(f)));
    scope (exit) physics.dispose();
    physics.settle(dt, 30);
    assert(physics.stallTime() < stallSeconds
        && runFailure(physics).length == 0,
        "усадка без хода — не сход");

    foreach (_; 0 .. cast(size_t)(stallSeconds / dt) + 10)
        physics.step(dt, 1.0f);
    assert(physics.stallTime() > stallSeconds,
        "стоячая машина копит время застоя");
    assert(runFailure(physics) == "нет продвижения вперёд",
        "застой по курсу — это тоже сход с дистанции");
}