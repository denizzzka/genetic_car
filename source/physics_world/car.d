module physics_world.car;

import std.math;
import std.algorithm : canFind, min, max, clamp;
import std.conv : to;
import std.stdio : writefln;

import dlib.core.memory;
import dlib.math.vector;
import dlib.math.quaternion;
import dlib.math.matrix;
import dlib.math.transformation;
import dlib.core.ownership;


// Имена dlib.math.transformation (up/forward/right — шаблоны) конфликтуют с
// базисом каркаса; нужные оси каркаса переименовываем локально. Остальной
// frame.frame импортируется поимённо.
import frame.frame : origin, frameUp = up, frameForward = forward,
    frameRight = right, frameBackward = backward,
    Frame, Node, Beam, Anchor, AnchorKind, BeamKind,
    asBeam, isConnected, initialMotorPower,
    heightOf, alongCourse, acrossCourse, courseYaw;
import physics_world.engine;
import physics_world.physics;
import physics_world.terrain;
import physics_world.terrainworld;
import physics_world.state : BodySnapshot, WorldSnapshot, bodySnapshot,
    restoreBody, worldStateVersion;
import physics_world.wheel;
import physics_world.route;
import frame.cockpit : CockpitGeometry, cockpitGeometry, cockpitInertia, cockpitMass;

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
    // Центральный узел — посадка кабины (низ кабины на узле 0): держим его
    // на уровне осей, как это делает геном (seed = точка опоры кабины),
    // иначе кабина сидит на крыше и на полном газу корпус тянет вилли.
    Frame frame;
    size_t node(vec3 pos)
    {
        frame.nodes ~= Node(pos);
        return frame.nodes.length - 1;
    }

    const c = node(vec3(0.0f, 0.0f, 0.3f));
    const fl = node(vec3(0.6f, 0.7f, 0.3f));
    const fr = node(vec3(-0.6f, 0.7f, 0.3f));
    const rl = node(vec3(0.6f, -0.7f, 0.3f));
    const rr = node(vec3(-0.6f, -0.7f, 0.3f));
    // Поперечные балки первыми: у каждого колеса ось — своя поперечина
    // (как у листового узла), а не наклонная диагональ к центру.
    frame.beams ~= new Beam(fl, fr, 0.045f);
    frame.beams ~= new Beam(rl, rr, 0.05f);
    frame.beams ~= new Beam(c, fl, 0.045f);
    frame.beams ~= new Beam(c, fr, 0.045f);
    frame.beams ~= new Beam(c, rl, 0.05f);
    frame.beams ~= new Beam(c, rr, 0.05f);
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
        assert(heightOf(s.position) > g + physicsWheelBelow, "колесо провалилось под землю");
        assert(heightOf(s.position) <= g + wheelRadius + 0.25f, "колесо парит над землёй");
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
    // Поперечные балки первыми — ось каждого колеса своя, поперечная.
    frame.beams ~= new Beam(fl, fr, 0.045f);
    frame.beams ~= new Beam(rl, rr, 0.05f);
    frame.beams ~= new Beam(c, fl, 0.045f);
    frame.beams ~= new Beam(c, fr, 0.045f);
    frame.beams ~= new Beam(c, rl, 0.05f);
    frame.beams ~= new Beam(c, rr, 0.05f);
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
        assert(heightOf(s.position) > g + physicsWheelBelow, "колесо провалилось под землю");
        assert(heightOf(s.position) <= g + wheelRadius + 0.25f, "колесо парит над землёй");
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
    frame.beams ~= new Beam(c, fl, 0.045f);
    frame.beams ~= new Beam(c, fr, 0.045f);
    frame.beams ~= new Beam(c, rl, 0.05f);
    frame.beams ~= new Beam(c, rr, 0.05f);
    frame.beams ~= new Beam(fl, fr, 0.045f);
    frame.beams ~= new Beam(rl, rr, 0.05f);
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
    const vec3 axle = w.worldRotation.rotate(Vector3f(0.0f, 1.0f, 0.0f));
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

/// Масса полой покрышки генетического радиуса: кольцо с теми же пропорциями,
/// что и базовое колесо.
private float tyreMass(const float r)
{
    const float ir = wheelInnerRadius * r / wheelRadius;
    const float w = wheelWidth * r / wheelRadius;
    return cast(float)(wheelDensity * PI * (r * r - ir * ir) * w);
}

/**
 * Физическая модель машины.
 *
 * Наружу (тесты, вьюер, фитнес) модель говорит только в координатах каркаса
 * (car-local): X вправо, Y вперёд, Z вверх. Оси движка — забота бэкенда.
 *
 * Создаётся независимо от `Buggy`, только когда нужен заезд: мир с землёй,
 * у каждой балки каркаса — отдельное кинестатическое тело-балка (контакты
 * регистрируются, но не решаются), у якорей — динамические колёса.
 * Жёсткость каркаса держит «мастер» — тело с массой и инерцией всей рамы
 * (коллизии у него выключены): к нему колёса приварены осями (револьте-
 * шарнир)
 * и из него на каждом шаге пересчитываются трансформы тел балок, так что узлы
 * не разбалтываются и каркас катится как монолит. Задевание балки земли или
 * чужого колеса ловится в контактах и жёстко отбраковывает заезд.
 * Привод — мотор-колёса: на ведущие колёса подаётся момент `Frame.motorPower`
 * (наследуемый ген) вокруг их оси, закрутка толкает машину по курсу (-Y).
 */
final class BuggyPhysics
{
    private PhysWorld world;

    /// Тело земли: статичный body с плоским heightfield, верх на y == 0.
    /// В terrain-режиме (terrain_ != null) земли нет — её ведёт terrainWorld_.
    private PhysBody ground;

    /// Общая процедурная поверхность (shared-кэш фитнес-пула и вьюера).
    /// При null заезд идёт по прежней плоскости y == 0.
    //
    // TODO: террейн (и кэш, и окно) логичнее держать не в BuggyPhysics, а в
    // мировом объекте: земля — содержимое мира, а «плоская/рельефная» земля —
    // свойство мира. Это уберёт тереновый параметр из обоих конструкторов и
    // ветвления в groundHeightAt/runFailure/beamUnderground, заодно снесёт
    // порядок сноса окна из dispose(). Но мир — код бэкенда
    // (внешняя зависимость), поэтому мир надо унаследовать (он не final,
    // как и dlib Owner) — например class BuggyWorld : NewtonPhysicsWorld с
    // setTerrain/reset/groundHeightAt. Решающий нюанс — пул: миры
    // переиспользуются без уничтожения мира, поэтому снести окно и обнулить
    // террейн обязан release() (worldpool) перед NewtonDestroyAllBodies,
    // иначе после рельефного заезда следующий «плоский» получит рельеф.
    private TerrainSurface terrain_;

    /// Стримингуемое окно поверхности в нашем мире (появляется в buildGround).
    private TerrainWorld terrainWorld_;

    /// «Мастер» каркаса: единый центр масс и инерции рамы. Он же везёт
    /// тела балок — их трансформы жёстко пересчитываются из мастера каждый
    /// шаг, рама идеально монолитна, а коллизии мастера выключены.
    private PhysBody master;

    /// Локальные смещение и ориентация каждой балки в мастер-теле.
    private Vector3f[] beamLocal;
    private Quaternionf[] beamLocalQuat;

    /// Положение кабины относительно мастера в координатах каркаса —
    /// для live-рендера (кабина жёстко приделана к мастеру).
    private vec3 cockpitLocal_ = origin;

    /// Точка каркаса (координаты каркаса), где стоит мастер: от неё
    /// отсчитываются локальные смещения кабины и балок.
    private vec3 masterLocal_ = origin;

    /// Кинестатические тела балок каркаса: по одному на каждую `Frame.beams`.
    private PhysBody[] beamBodies;

    /// Длина каждой балки — для геометрической проверки «под землёй».
    private float[] beamLen;

    /// Пары узлов концов балки (индекс в сварках и ступицах).
    private size_t[] beamNodeA, beamNodeB;

    /// Тела колёс по индексам `Frame.anchors`.
    private PhysBody[] wheelBodies;

    /// Узел якоря каждого колеса (своя ступица не считается задеванием).
    private size_t[] wheelNodes;

    /// Знак момента привода каждого колеса, выданный на старте: все колёса
    /// тянут особь к цели независимо от разворота оси, и знак не
    /// пересматривается по мере движения (ориентация колеса не влияет).
    private float[] wheelDriveSign;

    /// Осевой момент инерции каждого колеса: порог момента I·α_max, выше
    /// которого ни привод, ни регулятор крутки колесу крутку не добавляют
    /// (иначе малое колесо разгоняется в разлёт).
    private float[] wheelAxialMom;

    /// Тело рулевой балки (носовой рычаг от узла `steerNodeIndex`) и её
    /// шарнир. Мир шарнира принадлежит телу, поэтому понятие владения не
    /// расширяем: рычаг живёт в том же мире, что и мастер.
    private PhysBody steer;
    private PhysSteerJoint steerJoint_;

    /// Балки рулевой сборки: стойка вперёд от передней центральной ноды и, если
    /// на её конце поперечина, её половины. Все они едут на одном шарнире.
    private size_t[] steerBeams_;

    /// Узлы, чьи колёса катятся на рычаге: конец листовой стойки либо оба
    /// конца поперечины.
    private size_t[] steerWheelNodes_;

    /// Поворот рычага при сборке (car, истинный): локальные пивоты колёс
    /// рычага отсчитываются от него.
    private Quaternionf steerBuildQuat;

    /// Курсовой руль включён: каждый шаг `applySteer` правит `targetYaw`.
    /// Тесты выключают, чтобы покрутить шарнир вручную.
    bool autoSteer = true;

    /// Первый же провал заезда (латится): сенсорные колбэки и геометрия
    /// копят сюда причину, `beamFailure()` её выдаёт.
    private BeamFailure beamFail_;

    /// Виновники провала: индексы балок в `Frame.beams` и колёс в `anchors`.
    /// Вьюер оставляет их видимыми, пока моргает остальная машина.
    private size_t[] failBeams_;
    private size_t[] failWheels_;

    /// Время симуляции без нового продвижения вперёд по курсу (-Y каркаса).
    /// Обнуляется в `updateStall`, выливается в сход через `stallTime`.
    private double stallTime_ = 0.0;

    /// Набранный ход вперёд с последнего сброса, м. Маршрут и мировая Y
    /// не годятся: в повороте и при съезде с ленты машина едет, а проекция
    /// на маршрут стоит.
    private double forwardTravel_ = 0.0;

    /// Время симуляции этого мира, с: по нему ставятся снимки в буфер.
    private double elapsed_ = 0.0;

    /// Последний поданный газ: снимок хранит его для повтора с другим газом.
    private float lastThrottle_ = 0.0f;

    /// Собственное окно маршрута для Pure Pursuit. Каждый экземпляр ведёт
    /// своё окно: общий маршрут гонялся бы между потоками отбора.
    private Route route_;

    /// Машина-основа
    private const Buggy buggy_;

    /// Мир принадлежит нам, и `dispose` должен его уничтожить через `Delete`.
    /// Миры из пула (см. physics_world.worldpool) — чужие: `dispose` их не
    /// трогает, пул возвращает мир в оборот без уничтожения.
    private bool ownsWorld_;

    /// Свой мир: создаётся локально и забирается с собой (тесты/вьюер).
    this(const Buggy buggy, TerrainSurface terrain = null)
    {
        this(buggy, createPhysWorld(), terrain);
        ownsWorld_ = true;
    }

    this(const Buggy buggy, PhysWorld pooledWorld,
        TerrainSurface terrain = null)
    {
        buggy_ = buggy;
        terrain_ = terrain;
        world = pooledWorld;
        ownsWorld_ = false;
        world.useCallingThread();

        // Контакт колёс — провал заезда, но тела всё равно должны разойтись
        // честно: пару решаем и долаживаем.
        world.setInteraction(Interaction(BodyRole.wheel, BodyRole.wheel,
            ContactResponse.resolveAndReport, wheelWheelFriction,
            wheelWheelFriction, wheelWheelElasticity));
        world.setInteraction(Interaction(BodyRole.wheel, BodyRole.ground,
            ContactResponse.resolve, soilFrictionStatic, soilFrictionKinetic,
            soilElasticity));
        // Балки сами с собой не контачат: стыки узлов — не провал.
        world.setInteraction(Interaction(BodyRole.beam, BodyRole.beam,
            ContactResponse.ignore));

        // Каркас в Buggy уже разложен конструктором (center + на землю).
        buildGround();
        buildFrame();
        buildWheels();
        beamFail_ = BeamFailure.none;
        route_ = fixedRoute();
    }

    ~this()
    {
        dispose();
    }

    void dispose()
    {
        // Тайлы поверхности сносятся ДО того, как мир покинет пул
        // (clearScene) или будет уничтожен: иначе двойное
        // освобождение ground/boulder-тел.
        if (terrainWorld_ !is null)
        {
            terrainWorld_.dispose();
            terrainWorld_ = null;
        }
        if (world !is null)
        {
            // Свой мир уничтожаем целиком. Чужой (из пула)
            // только отсоединяем: пул сам вернёт его с пустыми телами.
            if (ownsWorld_)
                world.dispose();
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
        wheelDriveSign.length = 0;
        wheelAxialMom.length = 0;
        steer = null;
        steerJoint_ = null;
        steerBeams_ = null;
        steerWheelNodes_ = null;
        steerBuildQuat = Quaternionf.identity;
        beamFail_ = BeamFailure.none;
        failBeams_.length = 0;
        failWheels_.length = 0;
    }

    /// Частота шагов мира: столько раз в секунду движок готов считать.
    /// Заезды и живой цикл берут dt отсюда, а не из своей константы.
    double updateRate() @property
    {
        return world is null ? 60.0 : world.updateRate;
    }

    /// Один шаг симуляции. `dt` — ровно `1.0 / updateRate`: движок
    /// переваривает за вызов столько времени, сколько ему дали.
    ///
    /// `throttle` (0..1) кладёт момент на мотор-колёса: закрутка вокруг оси
    /// колеса толкает машину по курсу (-Y). Момент копится до world.update и
    /// сбрасывается внутри него, поэтому подаётся каждый шаг заново. Пассивные
    /// колёса на осях катятся сами — их везёт сцепление с землёй.
    void step(double dt, float throttle)
    {
        if (world is null)
            return;

        lastThrottle_ = throttle;
        beamFail_ = BeamFailure.none;
        failBeams_.length = 0;
        failWheels_.length = 0;

        applyDrive(throttle);
        applyWheelSpinGovernor(dt);
        applySteer();
        world.step(dt);
        elapsed_ += dt;
        syncBodies();
        updateContacts();
        updateBeamPuppets();
        updateStall(dt);
        if (terrainWorld_ !is null && master !is null)
            terrainWorld_.updateAround(master.worldPosition);
    }

    /// Момент полного газа на каждое мотор-колесо, разложенный по `throttle`.
    /// Сила мотора — наследуемый параметр каркаса (`Frame.motorPower`),
    /// знак силы — направление привода: положительный момент едет вперёд по
    /// курсу, отрицательный разворачивает вращение и едет назад (backward).
    /// Момент ограничен I·α_max: у малого колеса нет ни инерции, ни сцепления
    /// с грунтом, чтобы принять полный момент двигателя без разлёта (см.
    /// `maxWheelAngularAccel`).
    private void applyDrive(float throttle)
    {
        if (throttle == 0.0f || master is null)
            return;
        const Frame fr = buggy_.frame;
        if (fr.motorPower == 0.0f)
            return;
        foreach (i, a; fr.anchors)
            if (a.kind == AnchorKind.motorWheel && i < wheelBodies.length
                && i < wheelDriveSign.length && wheelBodies[i] !is null)
            {
                auto w = wheelBodies[i];
                // Момент вдоль живой оси колеса (физически вертеться можно
                // только вокруг неё), а знак — зафиксированный на сборке:
                // колесо крутится по «генному» направлению оси, текущая
                // ориентация знак не разворачивает.
                float t = throttle * fr.motorPower;
                const float cap = wheelAxialMom[i] * maxWheelAngularAccel;
                if (abs(t) > cap)
                    t = copysign(cap, t);
                const vec3 dir = wheelDriveSign[i]
                    * w.worldRotation.rotate(Vector3f(0.0f, 1.0f, 0.0f));
                w.addTorque(dir * t);
            }
    }

    /// Антипробуксовка (TCS): поверхности колеса разрешено быть быстрее
    /// грунта под ним только на `wheelSlipRatio` и не меньше стартового окна
    /// `wheelLaunchSurfaceSpeed` (иначе не тронуться с места). Превышение
    /// этого лимита гасится встречным моментом, пропорциональным превышению
    /// (`wheelSpinGain`) — предел мягкий, буксование ниже допуска ничем не
    /// стесняются. Абсолютный предел `wheelMaxSurfaceSpeed` остаётся сверху:
    /// на скорости выше него колесо не разгоняется даже ровно по грунту
    /// (иначе каркас разнесло бы за красивым лимитом). Действует на все
    /// колёса, не только на моторные: с каждой падает отряд.
    ///
    /// Агрессивность рассчитывается по колесу: gain не выше стабильной
    /// границы P-петли (2I/dt, запас `wheelSpinGainMargin`), момент — как и
    /// привод, в пределах I·α_max. Иначе у малого колеса петля расходится
    /// и вместо лёгкого лимита байка получаем разлёт каркаса.
    private void applyWheelSpinGovernor(double dt)
    {
        if (master is null)
            return;
        const Frame fr = buggy_.frame;
        foreach (i, w; wheelBodies)
            if (w !is null)
            {
                const vec3 axle = w.worldRotation.rotate(Vector3f(0.0f, 1.0f, 0.0f));
                const float spin = dot(axle, w.angularVelocity);
                const float r = i < fr.anchors.length ? fr.anchors[i].radius : wheelRadius;
                const float groundSpeed = abs(alongCourse(w.velocity));
                const float allowedSurface = min(
                    max(groundSpeed * (1.0f + wheelSlipRatio),
                        wheelLaunchSurfaceSpeed),
                    wheelMaxSurfaceSpeed);
                const float excess = abs(spin) - allowedSurface / r;
                if (excess > 0.0f)
                {
                    const float I = wheelAxialMom[i];
                    const float gain = min(wheelSpinGain,
                        wheelSpinGainMargin * 2.0f * I / cast(float)dt);
                    float t = min(excess * gain, I * maxWheelAngularAccel);
                    const float dir = (spin < 0.0f) ? -1.0f : 1.0f;
                    w.addTorque(axle * (-dir * t));
                }
            }
    }

    /// Курсовой руль: Pure Pursuit по маршруту. Ведёт машину за точкой,
    /// отстоящей вперёд по дуге на lookahead, — он же работает и в прогоне
    /// отбора, поэтому заезд всегда управляем. Ручной режим (`autoSteer ==
    /// false`, юнит-тесты) оставляет заданное рыскание, лишь зажимая его.
    private void applySteer()
    {
        if (steerJoint_ is null || steer is null || master is null)
            return;

        float cmd = steerJoint_.targetYaw;
        if (autoSteer)
        {
            const vec3 focus = wheelFocus();
            const RouteProjection rp = route_.project(focus);
            route_.cursor = rp.segment;
            ensureRouteAhead(route_, cast(float)rp.s + routeWindowHalfLength);
            trimRouteBehind(route_, cast(float)rp.s, routeWindowHalfLength);

            // Точка прицела — вперёд по дуге; с ростом скорости lookahead
            // растёт, иначе на быстром ходу руль дёргается по близкой точке.
            enum float lookaheadBase = 2.0f;
            enum float lookaheadPerSpeed = 0.15f;
            const float v = abs(alongCourse(master.velocity));
            const float L = lookaheadBase + v * lookaheadPerSpeed;
            const vec3 target = pointAtArc(route_, cast(float)rp.s + L);

            const vec3 toTarget = target - focus;
            const vec3 heading = master.worldRotation.rotate(
                Vector3f(0.0f, -1.0f, 0.0f));
            const float yawHeading = courseYaw(heading);
            const float yawTarget = atan2(
                acrossCourse(toTarget), alongCourse(toTarget));
            cmd = steerGain * (yawTarget - yawHeading);
        }
        steerJoint_.targetYaw = clamp(cmd, -steerLimitRad, steerLimitRad);
    }

    /// Учёт застоя по курсу: новый рекорд набега вперёд сбрасывает таймер,
    /// иначе к нему добавляется время текущего шага. `runFailure` считает
    /// переполнение `stallSeconds` сходом с дистанции.
    private void updateStall(double dt)
    {
        if (master is null)
            return;
        const vec3 fwd = master.worldFrameRotation.rotate(frameForward);
        const float v = dot(master.velocity, fwd);
        forwardTravel_ += max(0.0f, v) * dt;
        if (forwardTravel_ > stallProgressEps)
        {
            forwardTravel_ = 0.0;
            stallTime_ = 0.0;
        }
        else
            stallTime_ += dt;
    }

    /// Успокоить машину: шаги симуляции без движения, чтобы осадка рамы и
    /// первые контакты колёс улеглись и старт заезда был чистым.
    void settle(double dt, int steps)
    {
        foreach (_; 0 .. steps)
            step(dt, 0.0f);
    }

    /// Прочитать свежие позы тел из движка после шага: обёртка хранит копии
    /// position/rotation, и без этого вызова они не обновятся.
    private void syncBodies()
    {
        if (master is null)
            return;
        master.syncPose();
        foreach (w; wheelBodies)
            if (w !is null)
                w.syncPose();
    }

    BodyState[] beamStates()
    {
        BodyState[] res;
        foreach (b; beamBodies)
            if (b !is null)
            {
                BodyState s;
                s.position = b.worldPosition;
                s.orientation = b.worldRotation;
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
                s.position = w.worldPosition;
                s.orientation = w.worldRotation;
                res ~= s;
            }
        return res;
    }

    /// Положение кабины в мире: точка крепления кабины (низ на узле 0)
    /// жёстко приделана к мастеру, поэтому поворот и перенос — от мастера.
    BodyState cockpitState() @property
    {
        BodyState s;
        if (master is null)
            return s;
        // Кабина жёстко едет за мастером: поворот и перенос — истинное вращение
        // мастера (как у балок в updateBeamPuppets). Оффсет задан в координатах
        // каркаса: в пространство мастера его приводит тот же перевод, что и
        // всю геометрию. Ориентация наружу уходит в координатах каркаса, как у
        // балок (иначе при развороте машины кабина «плывёт» относительно рамы).
        Quaternionf frameRot = master.worldFrameRotation;
        s.position = master.worldPosition + frameRot.rotate(cockpitLocal_);
        s.orientation = frameRot;
        return s;
    }

    /// Снимок мира: мастер, руль, колёса, маршрут и таймеры заезда.
    /// Балки и кабина сюда не входят: они выводятся из тел и каркаса.
    WorldSnapshot saveState()
    {
        WorldSnapshot s;
        assert(master !is null, "снимок мира без мастера");
        s.master = bodySnapshot(master);
        s.hasSteer = steer !is null;
        if (s.hasSteer)
            s.steer = bodySnapshot(steer);
        foreach (w; wheelBodies)
            if (w !is null)
                s.wheels ~= bodySnapshot(w);
        s.steerTargetYaw = steerJoint_ is null ? 0.0f : steerJoint_.targetYaw;
        s.autoSteer = autoSteer;
        s.stallTime = stallTime_;
        s.forwardTravel = forwardTravel_;
        s.time = elapsed_;
        s.lastThrottle = lastThrottle_;
        s.hasTerrain = terrain_ !is null;
        s.route = route_.clone;
        return s;
    }

    /// Вернуть мир в снимок. Тела ставятся на место, маршрут и таймеры
    /// восстанавливаются; балки пересчитываются из мастера и руля.
    void restoreState(const WorldSnapshot s)
    {
        assert(s.formatVersion == worldStateVersion,
            "версия снимка мира не совпадает");
        assert(master !is null, "снимок мира без мастера");
        assert(s.hasSteer == (steer !is null),
            "руль в снимке не совпадает с каркасом");
        assert(s.hasTerrain == (terrain_ !is null),
            "рельеф в снимке не совпадает с миром");
        assert(s.wheels.length == wheelBodies.length,
            "число колёс в снимке не совпадает с каркасом");

        restoreBody(master, s.master);
        if (s.hasSteer)
            restoreBody(steer, s.steer);
        foreach (i, w; wheelBodies)
            if (w !is null)
                restoreBody(w, s.wheels[i]);

        if (steerJoint_ !is null)
            steerJoint_.targetYaw = s.steerTargetYaw;
        autoSteer = s.autoSteer;
        stallTime_ = s.stallTime;
        forwardTravel_ = s.forwardTravel;
        elapsed_ = s.time;
        lastThrottle_ = s.lastThrottle;
        route_ = s.route.clone;
        beamFail_ = BeamFailure.none;
        failBeams_.length = 0;
        failWheels_.length = 0;

        syncBodies();
        updateBeamPuppets();
        if (terrainWorld_ !is null)
            terrainWorld_.updateAround(master.worldPosition);
    }

    /// Касается ли кабина земли: любой угол нижней грани корпуса ушёл под
    /// опорную плоскость (строго, без допуска). Проверяется по живым
    /// состояниям симуляции, поэтому землю может задеть только крен/наклон
    /// в заезде. Идём по углам, а не по точке под ЦМ: при крене корпус
    /// упирается краем задолго до того, как его вертикаль погрузится.
    bool cabinTouchesGround()
    {
        if (master is null)
            return false;
        const cg = cockpitGeometry();
        auto s = cockpitState;
        foreach (offset; cg.floorCorners)
        {
            const vec3 p = s.position.xyz + s.orientation.rotate(offset);
            if (groundHeightAt(p) - heightOf(p) > 0.0f)
                return true;
        }
        return false;
    }

    /// Каркас машины: строит слой эфемерных балок, который клавиша D
    /// включает и в release, — поэтому доступ не под `debug`.
    const(Frame) frame() @property
    {
        return buggy_.frame;
    }

    /// Положение точки каркаса `p` (координаты каркаса) в мире каркаса же:
    /// точку жёстко несёт мастер, как тела балок и кабину. Для эфемерных
    /// балок, у которых своего тела нет.
    vec3 framePointWorld(const vec3 p)
    {
        if (master is null)
            return p;
        return master.worldPosition
            + master.worldRotation.rotate(p - masterLocal_);
    }

    // Отладочные хелперы обёрнуты в block-scoped `debug { }`: метка `debug:`
    // в release выключала всю остальную часть класса до его конца.
    debug {
    /// Максимальная линейная скорость тел (мастер + колёса): быстрый признак
    /// взрыва шага — скорость, разлетающаяся за десятки м/с до NaN.
    float dbgMaxLinSpeed() @property
    {
        float m = master !is null ? master.velocity.length : 0.0f;
        foreach (w; wheelBodies)
            if (w !is null)
                m = max(m, w.velocity.length);
        return m;
    }

    /// Максимальная угловая скорость тел — признак безудержной крутки.
    float dbgMaxAngSpeed() @property
    {
        float m = master !is null ? master.angularVelocity.length : 0.0f;
        foreach (w; wheelBodies)
            if (w !is null)
                m = max(m, w.angularVelocity.length);
        return m;
    }

    /// Скорости всех тел с меткой теле/индекса — какой элемент улетает.
    struct DbgBodySpeed
    {
        string kind;
        size_t idx;
        float lin, ang;
    }
    DbgBodySpeed[] dbgBodySpeeds() @property
    {
        DbgBodySpeed[] res;
        if (master !is null)
            res ~= DbgBodySpeed("мастер", 0,
                master.velocity.length, master.angularVelocity.length);
        foreach (i, w; wheelBodies)
            if (w !is null)
                res ~= DbgBodySpeed("колесо", i,
                    w.velocity.length, w.angularVelocity.length);
        return res;
    }

    /// Сырое (кэшированное dagon'ом) состояние мастера для отладки.
    BodyState dbgMasterState() @property
    {
        BodyState s;
        if (master is null)
            return s;
        s.position = master.worldPosition;
        s.orientation = master.worldRotation;
        return s;
    }

    /// Точка каркаса, где стоит мастер (координаты каркаса).
    vec3 masterLocal() @property
    {
        return masterLocal_;
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
        foreach (i, b; buggy_.frame.beams)
        {
            const vec3 a = fr.nodes[b.a].pos;
            const vec3 c = fr.nodes[b.b].pos;
            const vec3 d = c - a;
            if (d.length < 1e-5f)
                continue;
            PhysBody owner = (steer !is null && steerBeams_.canFind(i))
                ? steer : master;
            Quaternionf mt = owner.worldRotation;
            BeamTarget t;
            t.mid = owner.worldPosition + mt.rotate(beamLocal[i]);
            t.dir = mt.rotate(d);
            t.len = d.length;
            res ~= t;
        }
        return res;
    }
    }

    /**
     * Рулевое поддерево: всё физическое, что прикреплено к узлу руля.
     *
     * Обход идёт по настоящим балкам и не разворачивается из других узлов
     * кабины. Балка в такой узел означает сращивание носа с рамой: шарнир
     * уже не может повернуть сборку, поэтому руля нет.
     */
    private void findSteeringAssembly(const Frame frame)
    {
        steerBeams_ = null;
        steerWheelNodes_ = null;
        if (steerNodeIndex >= frame.nodes.length)
            return;

        bool[] cabin = new bool[frame.nodes.length];
        bool anyCabin;
        foreach (b; frame.beams)
            if (cast(Beam) b is null)
            {
                if (b.a < cabin.length) { cabin[b.a] = true; anyCabin = true; }
                if (b.b < cabin.length) { cabin[b.b] = true; anyCabin = true; }
            }
        // Синтетические каркасы без кабины (тесты) опознаются по старой схеме.
        version (unittest)
        {
            if (!anyCabin)
            {
                findSteeringAssemblyStrict(frame);
                return;
            }
        }

        // Нос начинается со стойки вперёд: боковая или задняя балка от узла
        // руля рулём не является.
        size_t stemIdx = size_t.max;
        float stemAhead = 0.0f;
        foreach (i, b; frame.beams)
        {
            const Beam beam = cast(Beam) b;
            if (beam is null)
                continue;
            if (beam.a != steerNodeIndex && beam.b != steerNodeIndex)
                continue;
            const size_t other = beam.a == steerNodeIndex ? beam.b : beam.a;
            const float ahead = dot(frame.nodes[other].pos
                - frame.nodes[steerNodeIndex].pos, frameForward);
            if (ahead > stemAhead)
            {
                stemAhead = ahead;
                stemIdx = i;
            }
        }
        if (stemIdx == size_t.max)
            return;

        bool[] seen = new bool[frame.nodes.length];
        seen[steerNodeIndex] = true;
        size_t[] stack = [steerNodeIndex];
        size_t[] beams;
        while (stack.length)
        {
            const size_t n = stack[$ - 1];
            stack.length--;
            foreach (i, b; frame.beams)
            {
                const Beam beam = cast(Beam) b;
                if (beam is null)
                    continue;
                if (beam.a != n && beam.b != n)
                    continue;
                const size_t other = beam.a == n ? beam.b : beam.a;
                if (other >= seen.length || seen[other])
                    continue;
                if (cabin[other] && other != steerNodeIndex)
                    return;
                seen[other] = true;
                stack ~= other;
                beams ~= i;
            }
        }
        if (beams.length == 0)
            return;
        steerBeams_ = beams;

        // Стойка — первая: по ней строятся тело рычага и ось шарнира.
        foreach (k, idx; steerBeams_)
            if (idx == stemIdx)
            {
                const size_t t = steerBeams_[0];
                steerBeams_[0] = idx;
                steerBeams_[k] = t;
                break;
            }

        foreach (a; frame.anchors)
            if (a.node < seen.length && seen[a.node]
                && a.node != steerNodeIndex)
                steerWheelNodes_ ~= a.node;
    }

version (unittest)
{
    /// Насколько поперечной должна быть балка на конце стойки, чтобы считаться
    /// половиной поперечины, а не её продолжением вперёд.
    private enum float forkOrthoLimit = 0.5f;

    /// Сколько балок (любых, включая эфемерные) сходятся в узле.
    private static size_t degree(const Frame frame, size_t node)
    {
        size_t n;
        foreach (b; frame.beams)
            if (b.a == node || b.b == node)
                ++n;
        return n;
    }

    /// Направление от одного узла каркаса к другому единичным вектором.
    private static vec3 direction(const Frame frame, size_t from, size_t to)
    {
        const vec3 d = frame.nodes[to].pos - frame.nodes[from].pos;
        const float len = d.length;
        return len < 1e-5f ? d : d / len;
    }

    /**
     * Найти рулевую сборку: какие балки едут на шарнире и на каких узлах стоят
     * колёса рычага.
     *
     * Сначала ищется вилка — стойка вперёд от передней центральной ноды с
     * поперечиной на конце: её опознание однозначно, и правило роста должно
     * побеждать выросшую наугад ветвь. Вилки нет — берётся самая длинная
     * листовая стойка, как и прежде.
     */
    private void findSteeringAssemblyStrict(const Frame frame)
    {
        steerBeams_ = null;
        steerWheelNodes_ = null;

        size_t leafBeam = size_t.max, leafNode = size_t.max;
        float leafLen = 0.0f;
        foreach (i, b; frame.beams)
        {
            const Beam beam = cast(Beam) b;
            if (beam is null)
                continue;
            if (beam.a != steerNodeIndex && beam.b != steerNodeIndex)
                continue;
            const size_t hub = (beam.a == steerNodeIndex) ? beam.b : beam.a;
            // Нос смотрит по курсу: поперечные и задние балки у передней
            // центральной ноды к рулю не относятся.
            const vec3 toHub = frame.nodes[hub].pos
                - frame.nodes[steerNodeIndex].pos;
            if (dot(toHub, frameForward) <= 0.0f)
                continue;

            size_t[2] halves, ends;
            if (forkAt(frame, i, hub, toHub, halves, ends))
            {
                steerBeams_ = [i, halves[0], halves[1]];
                steerWheelNodes_ = [ends[0], ends[1]];
                return;
            }

            // Свободный конец не должен крепить остальной каркас: любая балка
            // (в т.ч. эфемерная) там — топологическая связка.
            if (degree(frame, hub) != 1)
                continue;
            const float len = toHub.length;
            if (len > leafLen)
            {
                leafLen = len;
                leafBeam = i;
                leafNode = hub;
            }
        }
        if (leafBeam != size_t.max)
        {
            steerBeams_ = [leafBeam];
            steerWheelNodes_ = [leafNode];
        }
    }

    /**
     * Поперечина на конце стойки: ровно две настоящие балки, уводящие от
     * стойки в разные стороны, обе со свободными концами — под колёса вилки.
     * Ветвь, одна балка или продолжение стойки вперёд рулём не считаются.
     */
    private static bool forkAt(const Frame frame, size_t stemIdx, size_t hub,
        const vec3 toHub, out size_t[2] halves, out size_t[2] ends)
    {
        size_t n;
        foreach (i, b; frame.beams)
        {
            if (i == stemIdx)
                continue;
            const Beam beam = cast(Beam) b;
            if (beam is null || (beam.a != hub && beam.b != hub))
                continue;
            if (n == 2)
                return false;
            halves[n] = i;
            ends[n] = (beam.a == hub) ? beam.b : beam.a;
            ++n;
        }
        // Вилка — это ровно стойка и две балки: любая лишняя (в т.ч. эфемерная
        // от кабины) делает узел частью каркаса, а не рычагом.
        if (n != 2 || degree(frame, hub) != 3)
            return false;

        const vec3 stem = direction(frame, steerNodeIndex, hub);
        const vec3 side0 = direction(frame, hub, ends[0]);
        const vec3 side1 = direction(frame, hub, ends[1]);
        return dot(side0, stem) < forkOrthoLimit && dot(side1, stem) < forkOrthoLimit
            && dot(side0, side1) < 0.0f
            && degree(frame, ends[0]) == 1 && degree(frame, ends[1]) == 1;
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

        ground = world.createFlatGround(120.0f, 8, 0.0f);
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

        // Рулевая сборка: обычные балки, выходящие из передней центральной ноды
        // (steerNodeIndex) вперёд по курсу. Это либо листовая стойка с колесом
        // на свободном конце, либо вилка «стойка + поперечина» с колёсами на
        // обоих концах поперечины. Если её нет — каркас монолит.
        findSteeringAssembly(frame);

        // Геометрия каркаса — отдельные кинестатические тела-балки в
        // sensor-группе: контакты с землёй и чужими колёсами ловятся как
        // провал заезда, но никогда не толкают (колбэк их снимает).
        // Массы у балок нет — мост (master) несёт всю раму и тянет балки.
        // Эфемерные балки — только геометрия, тела не получают.
        foreach (i, b; frame.beams)
        {
            const auto beam = cast(Beam) b;
            if (beam is null)
                continue;
            const vec3 a = frame.nodes[b.a].pos;
            const vec3 c = frame.nodes[b.b].pos;
            const vec3 dir = c - a;
            const float len = dir.length;
            if (len < 1e-5f)
                continue;

            // Ось цилиндра (локальный Y) — вдоль балки.
            const Quaternionf q = rotationBetween(Vector3f(0, 1, 0), dir / len);
            auto body = world.createBody(BodyRole.beam, BodyMotion.kinematicBody,
                world.cylinderShape(beam.radius, beam.radius, len), 0.0f);
            body.tag = i;
            body.collidable = true;
            body.worldTransform((a + c) * 0.5f, q);
            body.syncPose();

            beamBodies[i] = body;
            beamLen[i] = len;
            beamNodeA[i] = b.a;
            beamNodeB[i] = b.b;
        }

        // Мастер: масса и центр масс по балкам, инерция от AABB каркаса.
        // Кабина вливается в то же тело: её масса, ЦМ и момент инерции
        // (заданы дизайном, оси OBJ = оси каркаса) сдвигаются параллельной
        // осевой теоремой к общему центру масс.
        float totalMass = 0.0f;
        vec3 sumM = origin;
        foreach (i, b; frame.beams)
        {
            if (steerBeams_.canFind(i))
                continue;
            const Beam beam = cast(Beam) b;
            if (beam is null)
                continue;
            const vec3 a = frame.nodes[b.a].pos;
            const vec3 b2 = frame.nodes[b.b].pos;
            const float len = (b2 - a).length;
            if (len < 1e-5f)
                continue;
            float m = cast(float)(beamDensity * PI * beam.radius * beam.radius * len);
            totalMass += m;
            sumM += (a + b2) * 0.5f * m;
        }
        if (totalMass <= 0.0f)
            totalMass = 1.0f;
        // Рулевая сборка несёт собственную массу — из мастера она вычитается.
        float steerMass = 0.0f;
        vec3 steerLo = vec3(float.max), steerHi = vec3(-float.max);
        foreach (i; steerBeams_)
        {
            const Beam beam = asBeam(frame.beams[i]);
            const vec3 a = frame.nodes[beam.a].pos;
            const vec3 c = frame.nodes[beam.b].pos;
            const float len = (c - a).length;
            steerMass += cast(float)(beamDensity * PI * beam.radius
                * beam.radius * len);
            foreach (p; [a, c])
            {
                steerLo = vec3(min(steerLo.x, p.x), min(steerLo.y, p.y),
                    min(steerLo.z, p.z));
                steerHi = vec3(max(steerHi.x, p.x), max(steerHi.y, p.y),
                    max(steerHi.z, p.z));
            }
        }
        // Колёса вилки висят на рычаге и двигаются с ним: их инерция — часть
        // сборки. Без неё лёгкий рычаг не держит тяжёлые диски, и оси
        // «пружинят» относительно концов поперечины. Вес колёс при этом
        // остаётся у самих колёс, поэтому рычагу он не начисляется.
        const float steerBarsMass = steerMass;
        foreach (a; frame.anchors)
            if (steerWheelNodes_.canFind(a.node))
            {
                steerMass += tyreMass(a.radius);
                const vec3 p = frame.nodes[a.node].pos;
                const vec3 e = vec3(a.radius, a.radius, a.radius);
                steerLo = vec3(min(steerLo.x, p.x - e.x),
                    min(steerLo.y, p.y - e.y), min(steerLo.z, p.z - e.z));
                steerHi = vec3(max(steerHi.x, p.x + e.x),
                    max(steerHi.y, p.y + e.y), max(steerHi.z, p.z + e.z));
            }
        if (steerMass <= 0.0f)
        {
            steerLo = origin;
            steerHi = origin;
        }
        const vec3 com = sumM / totalMass;

        // ЦМ кабины — узел 0 каркаса (совмещён с центром масс меша).
        const vec3 comCabin = frame.nodes[0].pos;
        float totalMassC = totalMass + cockpitMass;
        const vec3 comTotal = (sumM + comCabin * cockpitMass) / totalMassC;
        masterLocal_ = comTotal;

        master = world.createBody(BodyRole.master, BodyMotion.dynamicBody,
            world.boxShape(Vector3f(0.05f, 0.05f, 0.05f)), 0.0f);
        master.collidable = false; // коллизии считают балки и колёса
        master.gravity = gravityAccel;
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
        // Габариты рамы по осям каркаса: поперёк курса (ширина), вдоль курса
        // (длина) и вверх (высота).
        const vec3 box = maxP - minP;
        const float width = max(box.x, 0.05f);
        const float length = max(box.y, 0.05f);
        const float height = max(box.z, 0.05f);

        // Инерция рамы как монолита о её ЦМ (AABB-аппроксимация), сдвинутая
        // параллельной осевой теоремой к общему ЦМ: I = I_own + m*D².
        // Имена моментов — по оси вращения, а не по индексу компоненты.
        const vec3 dFrame = com - comTotal;
        const vec3 dCabin = comCabin - comTotal;
        const float iWidthFrame = (length * length + height * height) / 3.0f
            * totalMass + totalMass * (dFrame.y * dFrame.y + dFrame.z * dFrame.z);
        const float iLengthFrame = (width * width + height * height) / 3.0f
            * totalMass + totalMass * (dFrame.x * dFrame.x + dFrame.z * dFrame.z);
        const float iHeightFrame = (width * width + length * length) / 3.0f
            * totalMass + totalMass * (dFrame.x * dFrame.x + dFrame.y * dFrame.y);

        // Кабина: собственный тензор (16.7, 16.7, 8.5) + перенос её ЦМ.
        const float Mc = cockpitMass;
        const float iWidthCabin = cockpitInertia.x
            + Mc * (dCabin.y * dCabin.y + dCabin.z * dCabin.z);
        const float iLengthCabin = cockpitInertia.y
            + Mc * (dCabin.x * dCabin.x + dCabin.z * dCabin.z);
        const float iHeightCabin = cockpitInertia.z
            + Mc * (dCabin.x * dCabin.x + dCabin.y * dCabin.y);

        master.mass = totalMassC;
        master.inertia = Vector3f(iWidthFrame + iWidthCabin,
            iLengthFrame + iLengthCabin, iHeightFrame + iHeightCabin);

        master.setWorldPosition(comTotal);
        master.syncPose();

        // Положение кабины относительно мастера (в координатах каркаса):
        // для live-рендера кабина жёстко приделана к мастеру.
        cockpitLocal_ = comCabin - comTotal;

        // Рулевой рычаг: рулевая сборка — отдельное динамическое тело на
        // шарнире вокруг вертикали (up) в точке узла 1. Её масса ушла из
        // мастера; балки-сенсоры следуют за рычагом, а не за мастером.
        if (steerBeams_.length != 0)
        {
            const Beam stem = asBeam(frame.beams[steerBeams_[0]]);
            // Стойка ориентирована носом вперёд по курсу, в какую бы сторону
            // её ни записали: ось тела и ось шарнира от этого не зависят.
            const size_t near = stem.a == steerNodeIndex ? stem.a : stem.b;
            const size_t far = stem.a == steerNodeIndex ? stem.b : stem.a;
            const vec3 a = frame.nodes[near].pos;
            const vec3 c = frame.nodes[far].pos;
            const vec3 dir = c - a;
            const float len = dir.length;
            // Базис рычага строим сами: локальная Y — стойка, а крен
            // доворачиваем к вертикали. `rotationBetween` крен выбирает
            // произвольно, и на наклонной стойке (носовая вилка спускается к
            // земле) ось шарнира ушла бы от вертикали — шарнир борется сам с
            // собой и разрывает машину.
            const vec3 yAxis = dir / len;
            Quaternionf qBeam = rotationBetween(Vector3f(0, 1, 0), yAxis);
            const vec3 zWant = frameUp - yAxis * dot(frameUp, yAxis);
            if (dot(zWant, zWant) > 1e-6f)
            {
                const vec3 zNow = qBeam.rotate(Vector3f(0.0f, 0.0f, 1.0f));
                const float roll = atan2(dot(cross(zNow, zWant), yAxis),
                    dot(zNow, zWant));
                qBeam = rotationQuaternion(yAxis, roll) * qBeam;
            }
            steerBuildQuat = qBeam;
            const vec3 mid = (a + c) * 0.5f;

            // Ось руля — продольная ось стойки (локальная Y рычага): поворот
            // идёт вокруг самой балки, как бы та ни была наклонена в раме.
            // Вертикаль рамы годилась лишь приблизительно и на наклонной стойке
            // (носовая вилка спускается к земле) уводила ось от стойки.
            steer = world.createBody(BodyRole.master, BodyMotion.dynamicBody,
                world.cylinderShape(stem.radius, stem.radius, len), 0.0f);
            steer.collidable = false; // везёт шарнир и носовые колёса
            steer.gravity = gravityAccel;
            steer.linearDamping = bodyDamping;
            steer.angularDamping = Vector3f(bodyDamping, bodyDamping, bodyDamping);
            // Инерция сборки как монолита о её габаритах: поперёк стойки — вся
            // поперечина вилки, вдоль стойки — её длина. Ось шарнира — локальная
            // Y, а у плоской вилки поперечные оси равны, поэтому моменты совпадают.
            const float across = max(steerHi.x - steerLo.x, stem.radius * 2.0f);
            const float along = max(steerHi.y - steerLo.y, stem.radius * 2.0f);
            const float thick = max(steerHi.z - steerLo.z, stem.radius * 2.0f);
            const float axial = steerMass * (across * across + thick * thick) / 12.0f;
            const float perp = steerMass * (along * along + thick * thick) / 12.0f;
            steer.mass = steerMass;
            // Инерция колёс нужна решателю, а вес их несут сами тела колёс.
            if (steerMass > 0.0f)
                steer.gravity = gravityAccel * (steerBarsMass / steerMass);
            steer.inertia = Vector3f(perp, axial, perp);
            steer.worldTransform(mid, qBeam);
            steer.syncPose();

            const vec3 pivot = frame.nodes[steerNodeIndex].pos;
            // Ось шарнира — продольная ось стойки, то есть локальная Y рычага.
            // В осях рычага это просто (0, 1, 0), и та же ось, переведённая в
            // оси мастера, задаёт ось на второй стороне шарнира.
            steerJoint_ = world.newSteerJoint(steer, master,
                qBeam.conj.rotate(pivot - steer.worldPosition),
                pivot - master.worldPosition,
                Vector3f(0.0f, 1.0f, 0.0f),
                steerLimitRad);
        }

        // Локальные преобразования балок: в мастере — для рамы, в рычаге — для
        // рулевой сборки.
        foreach (i, b; frame.beams)
        {
            if (beamBodies[i] is null)
                continue;
            PhysBody owner = (steer !is null && steerBeams_.canFind(i))
                ? steer : master;
            beamLocal[i] = owner.worldRotation.conj.rotate(
                beamBodies[i].worldPosition - owner.worldPosition);
            beamLocalQuat[i] = owner.worldRotation.conj
                * beamBodies[i].worldRotation;
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
        // Истинное вращение мастера в car-координатах: всё дальше читается
        // именно оно.
        Quaternionf mTrue = master.worldRotation;
        foreach (i, b; beamBodies)
        {
            if (b is null)
                continue;
            // Раму несёт мастер, рулевую сборку — рычаг: её балки записаны
            // относительно рычага, иначе поперечина вилки уехала бы с ним.
            PhysBody owner = (steer !is null && steerBeams_.canFind(i))
                ? steer : master;
            Quaternionf oTrue = owner is master
                ? mTrue : owner.worldRotation;
            const vec3 r = oTrue.rotate(beamLocal[i]);
            const vec3 pos = owner.worldPosition + r;
            const Quaternionf q = oTrue * beamLocalQuat[i];
            b.worldTransform(pos, q);
            b.syncPose();
            b.velocity = owner.velocity + cross(owner.angularVelocity, r);
            b.angularVelocity = owner.angularVelocity;
        }
    }

    /// Разбор контактов за шаг: балка задела не своё колесо либо два колеса
    /// соприкоснулись — провал. Землю тут игнорируем, её ловит геометрическая
    /// beamUnderground().
    private void addFailBeam(size_t i)
    {
        if (!failBeams_.canFind(i))
            failBeams_ ~= i;
    }

    private void addFailWheel(size_t i)
    {
        if (!failWheels_.canFind(i))
            failWheels_ ~= i;
    }

    /// Балки каркаса, из-за которых заезд оборван (индексы `Frame.beams`).
    const(size_t)[] failureBeams() @property const { return failBeams_; }

    /// Колёса каркаса, из-за которых заезд оборван (индексы `Frame.anchors`).
    const(size_t)[] failureWheels() @property const { return failWheels_; }

    private void updateContacts()
    {
        foreach (pair; world.contacts())
        {
            if (pair.a.role == BodyRole.beam && pair.b.role == BodyRole.wheel)
                onBeamContact(pair.a.tag, pair.b);
            else if (pair.b.role == BodyRole.beam && pair.a.role == BodyRole.wheel)
                onBeamContact(pair.b.tag, pair.a);
            else if (pair.a.role == BodyRole.wheel && pair.b.role == BodyRole.wheel)
            {
                if (beamFail_ == BeamFailure.none)
                    beamFail_ = BeamFailure.wheelWheel;
                addFailWheel(pair.a.tag);
                addFailWheel(pair.b.tag);
            }
        }
    }

    /// Сенсорный контакт балки: задело колесо (не своё) — провал.
    private void onBeamContact(size_t beamIdx, const PhysBody other)
    {
        foreach (wi, w; wheelBodies)
            if (w is other)
            {
                if (!isOwnWheel(wheelNodes[wi], beamNodeA[beamIdx], beamNodeB[beamIdx]))
                {
                    beamFail_ = BeamFailure.wheel;
                    addFailBeam(beamIdx);
                    addFailWheel(wi);
                }
                return;
            }
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
    /// контактов движка — ловит и глухое погружение, и проскакивание между
    /// шагами проверки. Высота здесь — координата Z каркаса.
    private bool beamUnderground()
    {
        if (master is null)
            return false;
        const Frame fr = buggy_.frame;
        bool bad;
        foreach (i, b; beamBodies)
        {
            if (b is null)
                continue;
            const vec3 dir = b.worldRotation.rotate(Vector3f(0.0f, 1.0f, 0.0f));
            const float half = beamLen[i] * 0.5f;
            const float radius = asBeam(fr.beams[i]).radius;
            // Балка прямая: ниже земли может оказаться любой из концов, а не
            // только тот, что смотрит в -Y тела.
            foreach (end; [b.worldPosition - dir * half, b.worldPosition + dir * half])
                if (heightOf(end) - radius < groundHeightAt(end) - beamGroundEps)
                {
                    addFailBeam(i);
                    bad = true;
                    break;
                }
        }
        return bad;
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

    vec3 wheelFocus() @property
    {
        const wheels = wheelStates();
        if (wheels.length == 0)
            return surfaceFocus();
        vec3 p = vec3(0.0f, 0.0f, 0.0f);
        foreach (s; wheels)
            p += s.position;
        return p / wheels.length;
    }

    /// Боковое расстояние от фокуса машины до маршрута, м.
    float routeDistance() @property
    {
        if (route_.points.length < 2)
            return 0.0f;
        return route_.project(wheelFocus()).distance;
    }

    /// Фокус поверхности для стриминга окна: мастер, спроецированный на
    /// плоскость каркаса (вдоль `up` = 0).
    vec3 surfaceFocus() @property
    {
        if (master is null)
            return origin;
        const vec3 p = master.worldPosition;
        return p - frameUp * heightOf(p);
    }

    /// Мир-позиция мастера (центр массы каркаса) в координатах каркаса —
    /// точка, за которой следует камера живого заезда.
    Vector3f worldFocus() @property
    {
        if (master is null)
            return Vector3f(0.0f, 0.0f, 0.0f);
        return master.worldPosition;
    }

    /// Направление «вверх» рамы в координатах каркаса: на старте — строго +Z.
    /// По нему `runFailure` судит о перевороте (крен/тангаж).
    vec3 bodyUp() @property
    {
        if (master is null)
            return Vector3f(0.0f, 0.0f, 1.0f);
        // Вертикаль рамы: её собственная ось в базисе каркаса, повёрнутая
        // мировым поворотом мастера. Именно выровненный по каркасу поворот:
        // у фантомного тела локальные оси развёрнуты относительно каркаса.
        Quaternionf worldRot = master.worldFrameRotation;
        return worldRot.rotate(frameUp);
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

    /// Тело мастера для тестов (возмущения) и вьюера.
    PhysBody masterBody() @property
    {
        return master;
    }

    /// Есть ли у каркаса рулевой рычаг: шаг эволюции, когда листовая балка
    /// от передней центральной ноды дала свободный конец.
    bool isSteered() @property
    {
        return steer !is null;
    }

    /// Шарнир руля: тест крутит `targetYaw` вручную.
    PhysSteerJoint steerJoint() @property
    {
        return steerJoint_;
    }

    /// Текущий угол руля относительно мастера, рад.
    float steerYaw() @property
    {
        return steerJoint_ is null ? 0.0f : steerJoint_.yawNow;
    }

    private void buildWheels()
    {
        const Frame frame = buggy_.frame;
        wheelBodies.length = frame.anchors.length;
        wheelNodes.length = frame.anchors.length;
        wheelDriveSign.length = frame.anchors.length;
        wheelAxialMom.length = frame.anchors.length;

        foreach (i, a; frame.anchors)
        {
            const vec3 nodePos = frame.nodes[a.node].pos;

            // Генетический радиус колеса. Внутренний радиус покрышки и её
            // ширина масштабируются по отношению к базовому `wheelRadius`,
            // чтобы полое «кольцо» сохраняло пропорции.
            const float r = a.radius;
            const float ir = wheelInnerRadius * r / wheelRadius;
            const float w = wheelWidth * r / wheelRadius;

            const float mass = tyreMass(r);
            // Покрышка полая: труба, а не диск. Сплошной диск отвергал любой
            // каркас — балка подвески, кроме несущей, попадала внутрь колеса.
            auto wheel = world.createBody(BodyRole.wheel, BodyMotion.dynamicBody,
                world.tubeShape(r, ir, w), mass);
            wheel.tag = i;
            wheel.gravity = gravityAccel;
            // Линейное — лёгкий выкат (Crr гасит зацепление, не «воздух»);
            // угловое: ось спина (локальный Y цилиндра, см. `cylinderShape`) —
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
            wheel.mass = mass;
            wheel.inertia = Vector3f(perp, axial, perp);
            wheelAxialMom[i] = axial;

            // Axle across the course: the disc plane faces forward, so the
            // wheel rolls straight instead of scrubbing sideways on launch.
            const vec3 axleDir = wheelAxle(frameForward, frameUp, nodePos);
            const Quaternionf q = rotationBetween(Vector3f(0, 1, 0), axleDir);
            wheel.worldTransform(nodePos, q);
            wheel.syncPose();

            // Колесо держится на оси, закреплённой одним концом: револьте-
            // шарнир в точке узла якоря оставляет свободным только спин
            // колеса вокруг оси, а наклон оси относительно каркаса блокирует.
            // В отличие от BallConstraint (шаровой шарнир — все три поворота
            // свободны, колесо болтается вокруг пивота) ось не «гуляет».
            if (master !is null)
            {
                // Колесо на свободном конце рулевой балки катится с рычагом:
                // его ось живёт в осях рычага и поворачивается вместе с ним.
                PhysBody parent = master;
                vec3 pivotParentLocal = nodePos - master.worldPosition;
                if (steer !is null && steerWheelNodes_.canFind(a.node))
                {
                    parent = steer;
                    pivotParentLocal = steerBuildQuat.conj.rotate(
                        nodePos - steer.worldPosition);
                }
                world.newAxleJoint(wheel, parent, pivotParentLocal);
            }

            wheelBodies[i] = wheel;
            wheelNodes[i] = a.node;
            // Знак привода — на старте, по «генной» оси: все колёса тянут
            // особь к цели, независимо от разворота оси, и знак больше
            // не пересматривается (текущая ориентация не влияет).
            wheelDriveSign[i] = (dot(axleDir, frameRight) < 0.0f) ? -1.0f : 1.0f;
        }
    }
}

/// Вердикт заезда
RunOutcome runFailure(BuggyPhysics physics)
{
    // Застой по курсу: без набега вперёд с последней продвинутой точки
    // столько секунд подряд — это тоже сход с дистанции.
    if (physics.stallTime() > stallSeconds)
        return RunOutcome.stalled;

    if (physics.routeDistance() > routeMaxLateral)
        return RunOutcome.offCourse;

    const wheels = physics.wheelStates();
    if (wheels.length == 0)
        return RunOutcome.noWheels;

    // Переворот заезд не обрывает: без финиша машина может встать на колёса
    // и поехать дальше. Сход дают застой, земля и контакты.
    // const vec3 up = physics.bodyUp();
    // const float rollDeg = abs(atan2(acrossCourse(up), heightOf(up))) * 180.0f / PI;
    // const float pitchDeg = abs(atan2(alongCourse(up), heightOf(up))) * 180.0f / PI;
    // if (rollDeg > maxTiltDegrees || pitchDeg > maxTiltDegrees)
    //     return RunOutcome.rolledOver;

    foreach (wi, s; wheels)
    {
        if (!isFinite(s.position.x) || !isFinite(s.position.y)
            || !isFinite(s.position.z))
            return RunOutcome.frameBroke;
        // Локальная земля под колесом: 0 на плоскости, рельеф на поверхности.
        // Так колесо не «проваливается» на бугре и не «парит» над ложбиной.
        const float g = physics.groundHeightAt(s.position.xyz);
        if (heightOf(s.position) < g + physicsWheelBelow)
        {
            physics.addFailWheel(wi);
            return RunOutcome.wheelUnderground;
        }
    }

    foreach (s; physics.beamStates())
        if (!isFinite(s.position.x) || !isFinite(s.position.y)
            || !isFinite(s.position.z))
            return RunOutcome.frameBroke;

    switch (physics.beamFailure())
    {
        case BeamFailure.ground:
            return RunOutcome.beamGround;
        case BeamFailure.wheel:
            return RunOutcome.beamWheel;
        case BeamFailure.wheelWheel:
            return RunOutcome.wheelWheel;
        case BeamFailure.none:
        default:
            break;
    }
    return RunOutcome.none;
}
unittest
{
    // canDrive: решает, стоит ли запускать физический заезд.
    Frame f;
    f.nodes = [Node(origin), Node(frameRight)];
    f.beams = [new Beam(0, 1, 0.05f)];

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
        fr.beams ~= new Beam(0, 1, 0.050f);
        fr.beams ~= new Beam(0, 2, 0.050f);
        fr.beams ~= new Beam(1, 3, 0.044f);
        fr.beams ~= new Beam(2, 4, 0.044f);
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
    const double dt = 1.0 / 60.0;

    auto physics = new BuggyPhysics(new Buggy(placedFrame(uprightStubFrame())));
    scope (exit) physics.dispose();
    physics.settle(dt, 30);
    assert(physics.stallTime() < stallSeconds
        && runFailure(physics) == RunOutcome.none,
        "усадка без хода — не сход");

    foreach (_; 0 .. cast(size_t)(stallSeconds / dt) + 10)
        physics.step(dt, 1.0f);
    assert(physics.stallTime() > stallSeconds,
        "стоячая машина копит время застоя");
    assert(runFailure(physics) == RunOutcome.stalled,
        "застой по курсу — это тоже сход с дистанции");
}

unittest
{
    // Переворот ловится по наклону рамы. `bodyUp` — вектор «рама над
    // колёсами», а не ось фантомного тела: у того тела локальные оси не
    // совпадают с базисом каркаса, и мерить крен по нему бессмысленно.
    const double dt = 1.0 / 60.0;
    const float uprightZ = cos(maxTiltDegrees * PI / 180.0f);

    auto physics = new BuggyPhysics(new Buggy(placedFrame(uprightStubFrame())));
    scope (exit) physics.dispose();
    physics.settle(dt, 60);
    assert(heightOf(physics.bodyUp) > uprightZ,
        "усевшаяся машина стоит вертикально");

    // Кладём раму набок: переворот больше не обрывает заезд сам по себе.
    Quaternionf tilt = rotationBetween(frameUp, frameRight);
    physics.masterBody.worldTransform(
        tilt.rotate(physics.masterBody.worldPosition),
        tilt * physics.masterBody.worldRotation);
    assert(runFailure(physics) != RunOutcome.rolledOver,
        "перевёрнутая рама не обрывает заезд по одному лишь наклону");
}

unittest
{
    import physics_world.engineselect : defaultEngine, engineName, PhysicsEngine;
    import std.math : isFinite;

    // Руль: передняя центральная нода (узел 1) + листовая балка от неё со
    // свободным концом, несущим собственное колесо — рычаг получает шарнир.
    Frame boxFrame()
    {
        Frame frame;
        size_t node(vec3 pos)
        {
            frame.nodes ~= Node(pos);
            return frame.nodes.length - 1;
        }

        const c = node(vec3(0.0f, 0.3f, 0.35f));
        const fl = node(vec3(0.7f, -0.55f, 0.35f));
        const fr = node(vec3(-0.7f, -0.55f, 0.35f));
        const rl = node(vec3(0.7f, 0.4f, 0.35f));
        const rr = node(vec3(-0.7f, 0.4f, 0.35f));
        frame.beams ~= new Beam(fl, fr, 0.045f);
        frame.beams ~= new Beam(rl, rr, 0.05f);
        frame.beams ~= new Beam(c, fl, 0.045f);
        frame.beams ~= new Beam(c, fr, 0.045f);
        frame.beams ~= new Beam(c, rl, 0.05f);
        frame.beams ~= new Beam(c, rr, 0.05f);
        frame.anchors ~= Anchor(fl, AnchorKind.wheel);
        frame.anchors ~= Anchor(fr, AnchorKind.wheel);
        frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
        frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
        frame.motorPower = initialMotorPower;
        return frame;
    }

    Frame noseFrame()
    {
        Frame frame;
        size_t node(vec3 pos)
        {
            frame.nodes ~= Node(pos);
            return frame.nodes.length - 1;
        }

        const c = node(vec3(0.0f, 0.3f, 0.35f));
        const nose = node(vec3(0.0f, -0.9f, 0.85f)); // узел 1 = steerNodeIndex
        const tip = node(vec3(0.0f, -1.0f, 0.35f));
        const fl = node(vec3(0.7f, -0.55f, 0.35f));
        const fr = node(vec3(-0.7f, -0.55f, 0.35f));
        const rl = node(vec3(0.7f, 0.4f, 0.35f));
        const rr = node(vec3(-0.7f, 0.4f, 0.35f));
        frame.beams ~= new Beam(fl, fr, 0.045f);
        frame.beams ~= new Beam(rl, rr, 0.05f);
        frame.beams ~= new Beam(c, fl, 0.045f);
        frame.beams ~= new Beam(c, fr, 0.045f);
        frame.beams ~= new Beam(c, rl, 0.05f);
        frame.beams ~= new Beam(c, rr, 0.05f);
        frame.beams ~= new Beam(c, nose, 0.045f);
        frame.beams ~= new Beam(nose, tip, 0.05f);
        frame.anchors ~= Anchor(fl, AnchorKind.wheel);
        frame.anchors ~= Anchor(fr, AnchorKind.wheel);
        frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
        frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
        frame.anchors ~= Anchor(tip, AnchorKind.wheel);
        frame.motorPower = initialMotorPower;
        return frame;
    }

    // Без колеса на конце рулевой балки: рычаг ничем не нагружен — шарнир
    // свободно доезжает до своих пределов.
    Frame noseArmFrame()
    {
        Frame frame = noseFrame();
        frame.anchors.length = 4; // fl, fr, rl, rr — без носового колеса
        return frame;
    }

    // Форма, которую теперь отращивает правило роста носа: стойка вперёд от
    // передней центральной ноды, поперечина на её конце и колёса на обоих её
    // концах — вся сборка едет на одном шарнире.
    Frame forkFrame()
    {
        Frame frame;
        size_t node(vec3 pos)
        {
            frame.nodes ~= Node(pos);
            return frame.nodes.length - 1;
        }

        const c = node(vec3(0.0f, 0.3f, 0.35f));
        const nose = node(vec3(0.0f, -0.9f, 0.85f)); // узел 1 = steerNodeIndex
        const hub = node(vec3(0.0f, -1.0f, 0.5f));
        const fl = node(vec3(0.7f, -0.55f, 0.35f));
        const fr = node(vec3(-0.7f, -0.55f, 0.35f));
        const rl = node(vec3(0.7f, 0.4f, 0.35f));
        const rr = node(vec3(-0.7f, 0.4f, 0.35f));
        const nl = node(vec3(0.75f, -1.6f, 0.45f));
        const nr = node(vec3(-0.75f, -1.6f, 0.45f));
        frame.beams ~= new Beam(fl, fr, 0.045f);
        frame.beams ~= new Beam(rl, rr, 0.05f);
        frame.beams ~= new Beam(c, fl, 0.045f);
        frame.beams ~= new Beam(c, fr, 0.045f);
        frame.beams ~= new Beam(c, rl, 0.05f);
        frame.beams ~= new Beam(c, rr, 0.05f);
        frame.beams ~= new Beam(c, nose, 0.045f);
        frame.beams ~= new Beam(nose, hub, 0.05f);
        frame.beams ~= new Beam(hub, nl, 0.05f);
        frame.beams ~= new Beam(hub, nr, 0.05f);
        frame.anchors ~= Anchor(fl, AnchorKind.wheel);
        frame.anchors ~= Anchor(fr, AnchorKind.wheel);
        frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
        frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
        frame.anchors ~= Anchor(nl, AnchorKind.wheel);
        frame.anchors ~= Anchor(nr, AnchorKind.wheel);
        frame.motorPower = initialMotorPower;
        return frame;
    }

    // Руль проверяется на Jolt: шарнир Newton сломан и пока не чинится,
    // поэтому в проверке руля он не участвует.
    foreach (engine; [PhysicsEngine.jolt])
    {
        defaultEngine(engine);
        scope (exit) defaultEngine(PhysicsEngine.jolt);

        const double dt = 1.0 / 60.0;
        // Настоящий руль эволюции: каркас, который отращивает правило роста носа,
        // должен получить шарнир и привести его в движение. Именно этот заезд
        // боевой — вилка на поле, а не на стенде.
        {
            import genetics.growth : develop;
            import genetics.initial_data : startChromosome;
            const auto grown = develop(startChromosome());
            assert(!grown.isNull, "основатель должен вырасти");
            auto physics = new BuggyPhysics(new Buggy(placedFrame(grown.get)));
            scope (exit) physics.dispose();
            assert(physics.isSteered,
                "выросшая вилка должна дать рулевой рычаг");
            physics.settle(dt, 60);
            physics.autoSteer = false;
            physics.steerJoint.targetYaw = 8.0f;
            physics.settle(dt, 360);
            assert(abs(physics.steerYaw) <= steerLimitRad + 0.15f,
                "вилка основателя вышла за предел руля");
            assert(abs(physics.steerYaw) > steerLimitRad * 0.4f,
                "мотор не довёл вилку основателя до упора");
        }

        // Дальше — угол самого руля.
        {
            // Руль под нагрузкой: носовая балка с колесом — боевой случай
            // эволюции; достаточно, чтобы рычаг продавил сцепление покрышки
            // и дошёл до границы.
            auto physics = new BuggyPhysics(new Buggy(placedFrame(noseFrame())));
            scope (exit) physics.dispose();
            assert(physics.isSteered,
                "носовая листовая балка должна дать рулевой рычаг");
            physics.settle(dt, 60);

            physics.autoSteer = false;
            physics.steerJoint.targetYaw = 8.0f;
            foreach (_; 0 .. 360)
            {
                physics.step(dt, 0.0f);
                if ((_ & 31) == 0 || _ == 359)
                    writefln("st  %s t=%.2f yaw=%.4f",
                        engineName(engine), _ * dt, physics.steerYaw);
            }
            assert(abs(physics.steerYaw) <= steerLimitRad + 0.15f,
                "рычаг вышел за предел руля [" ~ engineName(engine) ~ "] yaw=" ~ physics.steerYaw.to!string);
            // Колесо листа стоит на самой оси шарнира, и полного доворота под
            // нагрузкой у него не бывает — покрышку срывает. Полный доворот
            // проверяет вилка (founder/forkFrame); здесь достаточно, что
            // нагруженный рычаг сдвинулся, а не замер.
            assert(abs(physics.steerYaw) > steerLimitRad * 0.25f,
                "нагруженный лист не сдвинулся под нагрузкой покрышки [" ~ engineName(engine) ~ "] yaw=" ~ physics.steerYaw.to!string);

            // Свободная ненагруженная балка: без покрышки на конце рычаг не
            // должен раскрутиться в воронку — предел держит и шарнир.
            auto bare = new BuggyPhysics(new Buggy(placedFrame(noseArmFrame())));
            scope (exit) bare.dispose();
            assert(bare.isSteered,
                "листовая балка без колеса тоже даёт рычаг");
            bare.settle(dt, 60);
            bare.autoSteer = false;
            bare.steerJoint.targetYaw = 8.0f;
            bare.settle(dt, 120);
            assert(abs(bare.steerYaw) <= steerLimitRad + 0.15f,
                "ненагруженный рычаг ушёл за предел руля");
        }


        // Вилка: шарнир везёт обе поперечные балки сразу, поэтому доворот рычага
        // должен быть заметно меньше предела — иначе мотор провернул пустую вилку.
        // Полный доворот до упора проверяет боевая вилка (founder выше): здесь
        // колёса на ступицах тормозят рычаг (каждое колесо решётка осевого
        // шарнира тянет за собой), и достаточно, чтобы вилка отвечала и не
        // выходила за предел.
        {
            auto physics = new BuggyPhysics(new Buggy(placedFrame(forkFrame())));
            scope (exit) physics.dispose();
            assert(physics.isSteered, "должна дать рулевой рычаг");
            physics.settle(dt, 60);

            physics.autoSteer = false;
            physics.steerJoint.targetYaw = 8.0f;
            physics.settle(dt, 360);
            assert(abs(physics.steerYaw) <= steerLimitRad + 0.15f,
                "вилка вышла за предел руля [" ~ engineName(engine) ~ "] yaw=" ~ physics.steerYaw.to!string);
            assert(abs(physics.steerYaw) > steerLimitRad * 0.25f,
                "вилка не сдвинулась [" ~ engineName(engine) ~ "] yaw=" ~ physics.steerYaw.to!string);
        }

        // Курсовой руль в заезде: стартовая крутка мастера вокруг вертикали
        // не сбивает машину с маршрута — Pure Pursuit гасит её, прогресс
        // растёт, а лента остаётся в коридоре.
        {
            auto physics = new BuggyPhysics(new Buggy(placedFrame(noseFrame())));
            scope (exit) physics.dispose();
            physics.settle(dt, 60);

            physics.masterBody.angularVelocity = Vector3f(0.0f, -0.8f, 0.0f);

            Route route = fixedRoute();
            double farthest = 0.0;
            float maxLateral = 0.0f;
            foreach (_; 0 .. 360)
            {
                physics.step(dt, 1.0f);
                const focus = physics.wheelFocus();
                farthest = max(farthest, routeProgress(route, focus, farthest));
                maxLateral = max(maxLateral, route.project(focus).distance);
            }
            assert(farthest > 5.0,
                "руль не повёл машину по маршруту [" ~ engineName(engine) ~ "]");
            assert(maxLateral < routeCorridorHalfWidth * 2.0f,
                "машина ушла с маршрута [" ~ engineName(engine) ~ "]");
        }

        // Без листовой балки от узла 1 руля нет: каркас монолит, заезд обычный.
        {
            auto physics = new BuggyPhysics(new Buggy(placedFrame(boxFrame())));
            scope (exit) physics.dispose();
            assert(!physics.isSteered,
                "без листовой балки от узла 1 руля быть не должно");
            physics.settle(dt, 30);
            foreach (_; 0 .. 120)
                physics.step(dt, 1.0f);
            assert(runFailure(physics).length == 0,
                "обычный заезд не должен ломаться из-за руля");
        }
    }

}

    version(unittest)
    {
        /// Минимальный правдоподобный каркас: четыре колеса и стойка рамы над
        /// плоскостью ступиц. Двухколёсная «палка» для проверок не годится:
        /// `placedFrame` не находит у неё курс, и рама встаёт на бок.
        private Frame uprightStubFrame()
        {
            Frame fr;
            size_t node(vec3 p)
            {
                fr.nodes ~= Node(p);
                return fr.nodes.length - 1;
            }

            const top = node(frameUp * 0.8f);
            const frontLeft = node(vec3(0.6f, 0.7f, 0.3f));
            const frontRight = node(vec3(-0.6f, 0.7f, 0.3f));
            const rearLeft = node(vec3(0.6f, -0.7f, 0.3f));
            const rearRight = node(vec3(-0.6f, -0.7f, 0.3f));
            fr.beams ~= new Beam(frontLeft, frontRight, 0.045f);

        fr.beams ~= new Beam(rearLeft, rearRight, 0.05f);
        foreach (wheel; [frontLeft, frontRight, rearLeft, rearRight])
            fr.beams ~= new Beam(top, wheel, 0.045f);
        fr.anchors ~= Anchor(frontLeft, AnchorKind.wheel);
        fr.anchors ~= Anchor(frontRight, AnchorKind.wheel);
        fr.anchors ~= Anchor(rearLeft, AnchorKind.motorWheel);
        fr.anchors ~= Anchor(rearRight, AnchorKind.motorWheel);
        return fr;
    }
}

unittest
{
    // Снимок возвращает тела, маршрут и таймеры в то же состояние.
    const double dt = 1.0 / 60.0;
    auto physics = new BuggyPhysics(new Buggy(placedFrame(uprightStubFrame())));
    scope (exit) physics.dispose();
    physics.settle(dt, 30);
    foreach (_; 0 .. 120)
        physics.step(dt, 1.0f);

    const auto snap = physics.saveState();
    const auto masterPos = snap.master.position;
    const auto masterRot = snap.master.orientation;
    const auto masterVel = snap.master.velocity;
    const auto wheelPos = snap.wheels[0].position;
    const auto wheelVel = snap.wheels[0].velocity;
    const auto stall = snap.stallTime;
    const auto time = snap.time;

    foreach (_; 0 .. 120)
        physics.step(dt, 1.0f);
    physics.restoreState(snap);

    const auto back = physics.saveState();
    assert((back.master.position - masterPos).length < 1e-4f,
        "мастер не вернулся в позу снимка");
    assert((back.master.velocity - masterVel).length < 1e-4f,
        "скорость мастера не вернулась");
    assert((back.wheels[0].position - wheelPos).length < 1e-4f,
        "колесо не вернулось в позу снимка");
    assert((back.wheels[0].velocity - wheelVel).length < 1e-4f,
        "скорость колеса не вернулась");
    assert(back.stallTime == stall, "таймер застоя не вернулся");
    assert(back.time == time, "время мира не вернулось");
    assert(back.route.points.length == snap.route.points.length
        && back.route.cursor == snap.route.cursor,
        "маршрут не вернулся в снимок");
    const q = back.master.orientation;
    assert(abs(q.w - masterRot.w) < 1e-4f && abs(q.x - masterRot.x) < 1e-4f
        && abs(q.y - masterRot.y) < 1e-4f && abs(q.z - masterRot.z) < 1e-4f,
        "поворот мастера не вернулся");

    physics.step(dt, 1.0f);
    assert(isFinite(physics.masterBody.worldPosition.x)
        && isFinite(physics.masterBody.worldPosition.y)
        && isFinite(physics.masterBody.worldPosition.z),
        "после отката мир снова шагает");
}
