module physics_world.car;

import std.math;
import std.algorithm : min, max;

import dlib.core.memory;
import dlib.math.vector;
import dlib.math.quaternion;
import dlib.math.utils;

import dmech;

import frame.frame;
import physics_world.physics;

/// Машина для отрисовки: каркас багги вместе с якорями (колёсами) и офсет,
/// приводящий каркас к удобному месту. Только данные — физика физикой
/// занимается отдельно (BuggyPhysics), когда нужен заезд.
class Buggy
{
    /// Каркас багги: узлы (позиции), балки, якоря.
    Frame frame;

    /// Смещение отображения, приводящее каркас к началу координат.
    /// Не меняет геометрию, применяется только при отрисовке.
    vec3 offset;

    this(Frame frame, vec3 offset)
    {
        this.offset = offset;
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

    auto physics = new BuggyPhysics(new Buggy(frame, vec3(0.0f)));
    scope (exit) physics.dispose();

    const double dt = 1.0 / 60.0;

    // Плоская земля, мотор-колёса: успокоить раму, затем заезд на полном газу.
    physics.settle(dt, 120);

    double avgY()
    {
        double y = 0.0;
        foreach (s; physics.wheelStates())
            y += s.position.y;
        return y / physics.wheelStates().length;
    }
    const double startY = avgY;

    // ~9 секунд заезда — машина должна остаться на земле,
    // не разлететься и не провалиться сквозь неё.
    foreach (i; 0 .. 540)
        physics.step(dt, 1.0f);

    foreach (s; physics.wheelStates())
    {
        assert(isFinite(s.position.x) && isFinite(s.position.y) && isFinite(s.position.z),
            "позиция колеса не конечна — машина разлетелась");
        assert(s.position.z > -0.1f, "колесо провалилось под землю");
        assert(s.position.z <= wheelRadius + 0.05f, "колесо парит над землёй");
    }
    foreach (s; physics.beamStates())
    {
        assert(isFinite(s.position.x) && isFinite(s.position.y) && isFinite(s.position.z),
            "позиция рамы не конечна — машина разлетелась");
    }

    // За ~9 секунд заезда машина должна заметно уехать вперёд по курсу.
    assert(-(avgY - startY) > 1.0,
        "мотор-колёса не везут машину — момент не передаётся раме");
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
    if (f.motorPower <= minMotorPower)
        return false;
    foreach (a; f.anchors)
        if (a.kind == AnchorKind.motorWheel)
            return true;
    return false;
}

/**
 * Физическая модель машины поверх dmech.
 *
 * Координаты — те же, что у каркаса (car-local): X вправо, Y вперёд, Z вверх.
 * Создаётся независимо от `Buggy`, только когда нужен заезд: мир с землёй,
 * у каждой балки каркаса — отдельный RigidBody с коллизиями (форма активна,
 * но не решается). Жёсткость каркаса держит «мастер» — тело с массой и
 * инерцией всей рамы: трансформы тел балок жёстко пересчитываются из него
 * на каждом шаге, так что узлы не разбалтываются и каркас катится как
 * монолит. Колёса у якорей приварены точкой (BallConstraint) к телу первой
 * балки узла. Столкновение балки с землёй или с чужим колесом регистрируется
 * beamFailure(): заезд обрывается как непройденный. Привод — мотор-колёса:
 * на ведущие колёса подаётся момент `Frame.motorPower` (наследуемый ген),
 * закрутка вокруг +X толкает машину по курсу (-Y).
 */
final class BuggyPhysics
{
    private PhysicsWorld world;

    /// Тело земли: статичный бокс, верхняя грань на z == 0.
    private RigidBody ground;

    /// Форма земли — для распознавания коллизий «балка об землю».
    private ShapeComponent groundShape;

    /// «Мастер» каркаса: единый центр масс и инерции рамы. Он же везёт
    /// тела балок — их трансформы жёстко пересчитываются из мастера каждый
    /// шаг, рама идеально монолитна, а солимер не разбалтывает узлы.
    private RigidBody master;

    /// Локальные смещение и ориентация каждой балки в мастер-теле.
    private Vector3f[] beamLocal;
    private Quaternionf[] beamLocalQuat;

    /// Тела балок каркаса: по одному RigidBody на каждую `Frame.beams`.
    private RigidBody[] beamBodies;

    /// Длина каждой балки — для геометрической проверки «под землёй».
    private float[] beamLen;

    /// Формы балок — для распознавания коллизий каркаса.
    private ShapeComponent[] beamShapes;

    /// Пары узлов концов балки (индекс в сварках и ступицах).
    private size_t[] beamNodeA, beamNodeB;

    /// Тела колёс по индексам `Frame.anchors`.
    private RigidBody[] wheelBodies;

    /// Формы колёс по индексам `Frame.anchors` — для распознавания коллизий.
    private ShapeComponent[] wheelShapes;

    /// Узел якоря каждого колеса (своя ступица не считается задеванием).
    private size_t[] wheelNodes;

    /// Сдвиг узлов каркаса в физических координатах: низ самого низкого
    /// колеса на z == 0 (как в вьюере).
    private vec3 posOffset;

    /// Машина-основа
    private const Buggy buggy_;

    this(const Buggy buggy)
    {
        buggy_ = buggy;
        world = New!PhysicsWorld(null, 1000);
        world.gravity = Vector3f(0.0f, 0.0f, -9.80665f); // Z вверх

        // Подъём: низ самого низкого колеса на z == 0.
        vec3 lift = vec3(0.0f);
        float minZ = float.max;
        foreach (a; buggy_.frame.anchors)
            minZ = min(minZ, buggy_.frame.nodes[a.node].pos.z);
        if (minZ < float.max)
            lift.z = wheelRadius - minZ;
        posOffset = lift;

        auto g = world.addStaticBody(Vector3f(0.0f, 0.0f, -0.5f));
        g.friction = 0.9f;
        ground = g;
        // GeomBox берёт ПОЛОВИННЫЕ габариты: размер 0.5 в Z + тело в z=-0.5
        // даёт верхнюю грань ровно на z == 0.
        groundShape = world.addShapeComponent(g,
            New!GeomBox(world, Vector3f(60.0f, 60.0f, 0.5f)),
            Vector3f(0.0f, 0.0f, 0.0f), 1.0f);

        buildFrame();
        buildWheels();
    }

    ~this()
    {
        dispose();
    }

    void dispose()
    {
        if (world !is null)
        {
            Delete(world);
            world = null;
        }
        ground = null;
        groundShape = null;
        master = null;
        beamBodies.length = 0;
        beamLen.length = 0;
        beamShapes.length = 0;
        beamNodeA.length = 0;
        beamNodeB.length = 0;
        wheelBodies.length = 0;
        wheelShapes.length = 0;
        wheelNodes.length = 0;
    }

    /// Один шаг симуляции. Фиксированный dt (~1/60) надёжно стабилен.
    ///
    /// `throttle` (0..1) кладёт момент на мотор-колёса: закрутка вокруг +X
    /// толкает машину по курсу (-Y). Момент копится до world.update и
    /// сбрасывается внутри него, поэтому подаётся каждый шаг заново. Пассивные
    /// колёса на осях катятся сами — их везёт сцепление с землёй.
    void step(double dt, float throttle)
    {
        if (world is null)
            return;

        applyDrive(throttle);
        world.update(dt);
        updateBeamPuppets();
    }

    /// Момент полного газа на каждое мотор-колесо, разложенный по `throttle`.
    /// Сила мотора — наследуемый параметр каркаса (`Frame.motorPower`).
    private void applyDrive(float throttle)
    {
        if (throttle == 0.0f || master is null)
            return;
        const Frame fr = buggy_.frame;
        if (fr.motorPower == 0.0f)
            return;
        foreach (i, a; fr.anchors)
            if (a.kind == AnchorKind.motorWheel && i < wheelBodies.length
                && wheelBodies[i] !is null)
                wheelBodies[i].applyTorque(
                    Vector3f(throttle * fr.motorPower, 0.0f, 0.0f));
    }

    /// Успокоить машину: шаги симуляции без движения, чтобы осадка рамы и
    /// первые контакты колёс улеглись и старт заезда был чистым.
    void settle(double dt, int steps)
    {
        foreach (_; 0 .. steps)
            step(dt, 0.0f);
    }

    BodyState[] beamStates()
    {
        BodyState[] res;
        foreach (b; beamBodies)
            if (b !is null)
            {
                BodyState s;
                s.position = b.position;
                s.orientation = b.orientation;
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
                s.position = w.position;
                s.orientation = w.orientation;
                res ~= s;
            }
        return res;
    }

    private void buildFrame()
    {
        const Frame frame = buggy_.frame;
        beamBodies.length = frame.beams.length;
        beamLen.length = frame.beams.length;
        beamShapes.length = frame.beams.length;
        beamNodeA.length = frame.beams.length;
        beamNodeB.length = frame.beams.length;
        beamLocal.length = frame.beams.length;
        beamLocalQuat.length = frame.beams.length;

        // Геометрия каркаса — отдельные тела-балки с коллизиями: столкновение
        // балки с землёй или чужим колесом ловится как провал заезда. Формы
        // активны, но не решаются (solve == false): ступичный проход осей и
        // ход рамы не должны толкаться контактами, а стабильность качения даёт
        // «мастер». Масса балок символическая — мост (master) несёт всю раму.
        foreach (i, b; frame.beams)
        {
            const vec3 a = frame.nodes[b.a].pos + posOffset;
            const vec3 c = frame.nodes[b.b].pos + posOffset;
            const vec3 dir = c - a;
            const float len = dir.length;
            if (len < 1e-5f)
                continue;

            auto body = world.addDynamicBody((a + c) * 0.5f, 0.0f);
            // Ось цилиндра (локальный Y) — вдоль балки.
            body.orientation = rotationBetween(Vector3f(0, 1, 0), dir / len);
            body.useGravity = false;
            body.friction = 0.0f;
            body.stopThreshold = 0.0f;

            auto shape = world.addShapeComponent(body,
                New!GeomCylinder(world, len, b.radius),
                Vector3f(0.0f, 0.0f, 0.0f), 0.1f);
            shape.active = true;
            shape.solve = false;

            beamBodies[i] = body;
            beamLen[i] = len;
            beamShapes[i] = shape;
            beamNodeA[i] = b.a;
            beamNodeB[i] = b.b;
        }

        // Мастер: масса и центр масс по балкам, инерция от AABB каркаса.
        float totalMass = 0.0f;
        vec3 sumM = vec3(0.0f);
        foreach (b; frame.beams)
        {
            const vec3 a = frame.nodes[b.a].pos + posOffset;
            const vec3 b2 = frame.nodes[b.b].pos + posOffset;
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

        master = world.addDynamicBody(com, 0.0f);
        master.stopThreshold = 0.0f;

        // AABB каркаса для грубой инерции рамы.
        vec3 minP = vec3(float.max), maxP = vec3(-float.max);
        foreach (n; frame.nodes)
        {
            const vec3 p = n.pos + posOffset;
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
        auto mshape = world.addShapeComponent(master,
            New!GeomBox(world, dims), Vector3f(0.0f, 0.0f, 0.0f), totalMass);
        // Мастер не участвует в коллизиях — их считают балки.
        mshape.active = false;
        mshape.solve = false;

        // Локальные преобразования балок в мастере.
        foreach (i, b; frame.beams)
        {
            if (beamBodies[i] is null)
                continue;
            beamLocal[i] = master.orientation.conj.rotate(
                beamBodies[i].position - master.position);
            beamLocalQuat[i] = master.orientation.conj * beamBodies[i].orientation;
        }
    }

    /// Пересчёт тел балок из мастера после шага симуляции: рама следует за
    /// своим центром масс как монолит. Скорости тоже приравниваются, чтобы
    /// контакты колёс считались против согласованного движения каркаса.
    private void updateBeamPuppets()
    {
        if (master is null)
            return;
        foreach (i, b; beamBodies)
        {
            if (b is null)
                continue;
            const vec3 r = master.orientation.rotate(beamLocal[i]);
            b.position = master.position + r;
            b.orientation = master.orientation * beamLocalQuat[i];
            b.linearVelocity = master.linearVelocity
                + cross(master.angularVelocity, r);
            b.angularVelocity = master.angularVelocity;
        }
    }

    /**
     * Обрыв заезда из-за каркаса: балка ударилась об землю или о колесо.
     *
     * Контакт балки со СВОЕЙ ступицей (колесом, приваренным к концу этой
     * балки) не считается задеванием — балка легитимно проходит через
     * отверстие оси своего колеса.
     */
    BeamFailure beamFailure()
    {
        if (world is null)
            return BeamFailure.none;

        foreach (ref m; world.manifolds)
        {
            for (uint i = 0; i < m.numContacts; ++i)
            {
                const c = &m.contacts[i];
                const s1 = c.shape1;
                const s2 = c.shape2;
                if (s1 is null || s2 is null)
                    continue;

                const bi = beamShapeIndex(s1);
                const bj = beamShapeIndex(s2);
                if (bi != size_t.max && (s2 is groundShape)
                    && c.point.z < -beamGroundEps)
                    return BeamFailure.ground;
                if (bj != size_t.max && (s1 is groundShape)
                    && c.point.z < -beamGroundEps)
                    return BeamFailure.ground;

                if (bi != size_t.max)
                {
                    const wj = wheelShapeIndex(s2);
                    if (wj != size_t.max && !isOwnWheel(bi, wj))
                        return BeamFailure.wheel;
                }
                if (bj != size_t.max)
                {
                    const wi = wheelShapeIndex(s1);
                    if (wi != size_t.max && !isOwnWheel(bj, wi))
                        return BeamFailure.wheel;
                }
            }
        }
        if (beamUnderground())
            return BeamFailure.ground;

        return BeamFailure.none;
    }

    /// Геометрическая проверка «рама под землёй»: низшая точка поверхности
    /// любой балки ниже `-beamGroundEps`. Не зависит от манифолдов — ловит
    /// и глухое погружение, и проскакивание между шагами проверки.
    private bool beamUnderground()
    {
        if (master is null)
            return false;
        const Frame fr = buggy_.frame;
        foreach (i, b; beamBodies)
        {
            if (b is null)
                continue;
            // Ось цилиндра — локальный Y; низшая точка балки над землёй.
            const vec3 dir = b.orientation * Vector3f(0.0f, 1.0f, 0.0f);
            const float half = beamLen[i] * 0.5f;
            const float low = (b.position.z - dir.z * half) - fr.beams[i].radius;
            if (low < -beamGroundEps)
                return true;
        }
        return false;
    }

    private size_t beamShapeIndex(const ShapeComponent s)
    {
        foreach (i, sh; beamShapes)
            if (s is sh)
                return i;
        return size_t.max;
    }

    private size_t wheelShapeIndex(const ShapeComponent s)
    {
        foreach (i, sh; wheelShapes)
            if (s is sh)
                return i;
        return size_t.max;
    }

    /// Своя ступица: колесо приварено к концу этой балки.
    private bool isOwnWheel(size_t beam, size_t wheel)
    {
        return wheelNodes[wheel] == beamNodeA[beam]
            || wheelNodes[wheel] == beamNodeB[beam];
    }

    private void buildWheels()
    {
        const Frame frame = buggy_.frame;
        wheelBodies.length = frame.anchors.length;
        wheelShapes.length = frame.anchors.length;
        wheelNodes.length = frame.anchors.length;

        foreach (i, a; frame.anchors)
        {
            const vec3 nodePos = frame.nodes[a.node].pos + posOffset;

            auto wheel = world.addDynamicBody(nodePos, 0.0f);
            // Ось цилиндра (локальный Y) — вдоль поперечной оси машины X.
            wheel.orientation = rotationBetween(Vector3f(0, 1, 0), Vector3f(1, 0, 0));
            wheel.friction = 0.9f;
            wheel.stopThreshold = 0.0f;

            float mass = cast(float)(wheelDensity * PI
                * (wheelRadius * wheelRadius - wheelInnerRadius * wheelInnerRadius)
                * wheelWidth);
            auto shape = world.addShapeComponent(wheel,
                New!GeomWheel(world, wheelWidth, wheelRadius, wheelInnerRadius),
                Vector3f(0.0f, 0.0f, 0.0f), mass);
            shape.active = true;
            shape.solve = true;

            // Колесо приварено точкой (BallConstraint) к мастер-каркасу в
            // точке узла якоря: свободно вращается вокруг своей оси, не мешая
            // качению. Стабильно, как в исходной однотелой схеме (мягкая
            // сварка с дефолтными параметрами), а жёсткий постоянный перенос
            // рамы даёт мастер; тела балок только передают геометрию.
            // Осевой HingeConstraint непригоден: AxisAngle в нём жёстко
            // блокирует вращение колеса (импульсы по перпендикулярным осям +
            // трение контакта запирают качение, машина почти не едет).
            if (master !is null)
            {
                Vector3f anchor =
                    master.orientation.conj.rotate(nodePos - master.position);
                Vector3f wheelAnchor =
                    wheel.orientation.conj.rotate(Vector3f(weldEps, 0.0f, 0.0f));
                auto weld = New!BallConstraint(world, master, wheel,
                    anchor, wheelAnchor);
                world.addConstraint(weld);
            }

            wheelBodies[i] = wheel;
            wheelShapes[i] = shape;
            wheelNodes[i] = a.node;
        }
    }
}

/// Вердикт заезда
string runFailure(BuggyPhysics physics)
{
    const wheels = physics.wheelStates();
    if (wheels.length == 0)
        return "не осталось колёс";

    foreach (s; wheels)
    {
        if (!isFinite(s.position.x) || !isFinite(s.position.y)
            || !isFinite(s.position.z))
            return "каркас разлетелся";
        if (s.position.z < physicsWheelBelow)
            return "колесо провалилось под землю";
        if (s.position.z > wheelRadius + physicsWheelLift)
            return "машина перевернулась";
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
    f.nodes = [Node(vec3(0.0f)), Node(vec3(0.0f, 1.0f, 0.0f))];
    f.beams = [Beam(0, 1, 0.05f)];

    f.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(!canDrive(f), "нет мотор-колеса — привода нет");

    f.anchors ~= Anchor(1, AnchorKind.motorWheel);
    f.motorPower = minMotorPower;
    assert(!canDrive(f), "момент на пороге не считается приводом");

    f.motorPower = minMotorPower + 1.0f;
    assert(canDrive(f), "мотор-колесо с заметным моментом — привод есть");
}
