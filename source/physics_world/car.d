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

    auto physics = new BuggyPhysics(frame);
    scope (exit) physics.dispose();

    const double dt = 1.0 / 60.0;

    // Уклон горки и успокоение: осадка рамы без движения.
    physics.setSlopeDeg(25.0f);
    physics.settle(dt, 120);

    double avgY()
    {
        double y = 0.0;
        foreach (s; physics.wheelStates())
            y += s.position.y;
        return y / physics.wheelStates().length;
    }
    const double startY = avgY;

    // ~9 секунд спуска — машина должна остаться на земле,
    // не разлететься и не провалиться сквозь неё.
    foreach (i; 0 .. 540)
        physics.step(dt, 0.0f);

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

    // За ~9 секунд спуска машина должна заметно укатиться по курсу.
    assert(-(avgY - startY) > 1.0,
        "машина не катится с горки — колёса на осях не катятся");
}

/**
 * Физическая модель машины поверх dmech.
 *
 * Координаты — те же, что у каркаса (car-local): X вправо, Y вперёд, Z вверх.
 * Создаётся независимо от `Buggy`, только когда нужен заезд: мир с землёй,
 * одна жёсткая рама (массы балок сведены в единый RigidBody-центр масс,
 * коллизии рамы выключены) и колёса у якорей, приваренные точкой
 * (BallConstraint) к раме у узла. Привод — гравитационный: наклоном `g`
 * (setSlopeDeg) моделируется горка, машина катится сама. Момента на колёсах
 * нет — на этом dmech он не передаёт движение раме.
 */
final class BuggyPhysics
{
    private PhysicsWorld world;

    /// Тело земли: статичный бокс, верхняя грань на z == 0.
    private RigidBody ground;

    /// Тело рамы: массы всех балок, инерция от AABB каркаса.
    private RigidBody chassis;

    /// Тела колёс по индексам `Frame.anchors`.
    private RigidBody[] wheelBodies;

    /// Сдвиг узлов каркаса в физических координатах: низ самого низкого
    /// колеса на z == 0 (как в вьюере).
    private vec3 posOffset;

    this(const Frame frame)
    {
        world = New!PhysicsWorld(null, 1000);
        world.gravity = Vector3f(0.0f, 0.0f, -9.80665f); // Z вверх

        // Подъём: низ самого низкого колеса на z == 0.
        vec3 lift = vec3(0.0f);
        float minZ = float.max;
        foreach (a; frame.anchors)
            minZ = min(minZ, frame.nodes[a.node].pos.z);
        if (minZ < float.max)
            lift.z = wheelRadius - minZ;
        posOffset = lift;

        auto g = world.addStaticBody(Vector3f(0.0f, 0.0f, -0.5f));
        g.friction = 0.9f;
        ground = g;
        // GeomBox берёт ПОЛОВИННЫЕ габариты: размер 0.5 в Z + тело в z=-0.5
        // даёт верхнюю грань ровно на z == 0.
        world.addShapeComponent(g, New!GeomBox(world, Vector3f(60.0f, 60.0f, 0.5f)),
            Vector3f(0.0f, 0.0f, 0.0f), 1.0f);

        buildChassis(frame);
        buildWheels(frame);
    }

    /// Уклон «горки»: силу тяжести разворачиваем так, чтобы появилась составляющая
    /// вдоль курса (+Y). Земля остаётся плоской — это тот же скат, но без
    /// перекашивания каркаса.
    void setSlopeDeg(float degrees)
    {
        const float a = degrees * PI / 180.0f;
        world.gravity = Vector3f(0.0f,
            -9.80665f * sin(a),
            -9.80665f * cos(a));
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
        chassis = null;
        wheelBodies.length = 0;
    }

    /// Один шаг симуляции. Фиксированный dt (~1/60) надёжно стабилен.
    ///
    /// Из пассивных колёс на осях и наклона `g` машина едет сама — привод
    /// моментом на этом dmech не передаёт движение раме (момент раскручивает
    /// колесо, но тяга гаснет о трение и связи). `throttle` сохранён для
    /// совместимости, газ не применяется.
    void step(double dt, float throttle)
    {
        if (world is null)
            return;

        world.update(dt);
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
        if (chassis !is null)
        {
            BodyState s;
            s.position = chassis.position;
            s.orientation = chassis.orientation;
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

    private void buildChassis(const Frame frame)
    {
        // Масса и центр масс по балкам (каждая — цилиндр с осью вдоль длины).
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

        chassis = world.addDynamicBody(com, 0.0f);
        chassis.stopThreshold = 0.0f;

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

        auto shape = world.addShapeComponent(chassis,
            New!GeomBox(world, dims), Vector3f(0.0f, 0.0f, 0.0f), totalMass);
        shape.active = false;
        shape.solve = false;
    }

    private void buildWheels(const Frame frame)
    {
        wheelBodies.length = frame.anchors.length;

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
            world.addShapeComponent(wheel,
                New!GeomWheel(world, wheelWidth, wheelRadius, wheelInnerRadius),
                Vector3f(0.0f, 0.0f, 0.0f), mass);

            // Колесо приварено точкой (BallConstraint) к раме у узла якоря:
            // свободно вращается вокруг своей оси, не мешая качению.
            // Осевой HingeConstraint непригоден: AxisAngle в нём жёстко
            // блокирует вращение колеса (импульсы по перпендикулярным осям +
            // трение контакта запирают качение, машина почти не едет).
            Vector3f anchor =
                chassis.orientation.conj.rotate(nodePos - chassis.position);
            Vector3f wheelAnchor =
                wheel.orientation.conj.rotate(Vector3f(weldEps, 0.0f, 0.0f));
            world.addConstraint(New!BallConstraint(world, chassis, wheel,
                anchor, wheelAnchor));

            wheelBodies[i] = wheel;
        }
    }
}