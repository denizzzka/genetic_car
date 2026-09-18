module physics_world.physics;

import std.math;
import std.algorithm : min, max;

import dlib.core.memory;
import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;
import dlib.math.utils;
import dlib.geometry.aabb;

import dmech;

import frame.frame;

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

    // Подъём по Z, как в viewer: низ самого низкого колеса на z == 0.
    vec3 offset = vec3(0.0f);
    float minZ = float.max;
    foreach (a; frame.anchors)
        if (frame.nodes[a.node].pos.z < minZ)
            minZ = frame.nodes[a.node].pos.z;
    offset.z = wheelRadius - minZ;

    auto physics = new CarPhysics(frame, offset);
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

/// Радиус колеса. Совпадает с визуальным тором: ShapeTorus(0.2f, 0.1f)
/// даёт внешний радиус 0.2 + 0.1 = 0.3.
enum float wheelRadius = 0.3f;

/// Внутренний радиус «отверстия» колеса (покрышка — полый цилиндр).
enum float wheelInnerRadius = 0.22f;

/// Ширина колеса (толщина покрышки, равна диаметру трубы тора).
enum float wheelWidth = 0.2f;

/// Плотность материала колеса, кг/м^3.
enum float wheelDensity = 400.0f;

/// Компенсация бага dmech: BallConstraint делает `dp.normalized` на разнице
/// якорей; при точно совпадающих якорях 0/0 даёт NaN. Сварочные узлы и шарниры
/// сводим на микро-отступ (всегда > 0), чтобы нормаль была корректной.
enum float weldEps = 1e-3f;

/// Плотность материала балок каркаса, кг/м^3.
enum float beamDensity = 150.0f;

/// Транспортное состояние тела: позиция и ориентация в координатах машины.
/// Совпадает с трансформацией Dagon-сущности под carRoot:
/// `entity.position = state.position; entity.rotation = state.orientation;`
struct BodyState
{
    Vector3f position;
    Quaternionf orientation;
}

/// Полый цилиндр («цилиндр с отверстием») для колеса. Ось — локальный Y.
///
/// Опорная функция совпадает с внешней поверхностью сплошного цилиндра,
/// поэтому коллизии считаются по внешнему радиусу; инерция — полого цилиндра.
/// Это без проблем для MPR: опорное отображение выпуклой оболочки то же.
final class GeomWheel: Geometry
{
    float height;
    float radius;
    float innerRadius;

    this(PhysicsWorld world, float h, float r, float ri)
    {
        super(world);
        type = GeomType.UserDefined;
        height = h;
        radius = r;
        innerRadius = ri;
    }

    override Vector3f supportPoint(Vector3f dir)
    {
        Vector3f result;
        float sigma = sqrt(dir.x * dir.x + dir.z * dir.z);

        if (sigma > 0.0f)
        {
            result.x = dir.x / sigma * radius;
            result.z = dir.z / sigma * radius;
        }
        result.y = sign(dir.y) * height * 0.5f;

        return result;
    }

    override Matrix3x3f inertiaTensor(float mass)
    {
        float r2 = radius * radius;
        float ri2 = innerRadius * innerRadius;
        float h2 = height * height;

        float perp = (3.0f * (r2 + ri2) + h2) / 12.0f * mass;
        float axial = 0.5f * (r2 + ri2) * mass;

        return matrixf(
            perp, 0.0f, 0.0f,
            0.0f, axial, 0.0f,
            0.0f, 0.0f, perp
        );
    }

    override AABB boundingBox(Vector3f position)
    {
        float rsum = radius + radius;
        float d = sqrt(rsum * rsum + height * height) * 0.5f;
        return AABB(position, Vector3f(d, d, d));
    }
}

/**
 * Физическая модель машины поверх dmech.
 *
 * Координаты — те же, что у каркаса (car-local): X вправо, Y вперёд, Z вверх.
 * Каркас — одна жёсткая рама: массы балок сведены в единый RigidBody-центр масс
 * (коллизии рамы выключены). Колёса — тела у якорей, приварены точкой
 * (BallConstraint) к раме у узла: свободно вращаются без навязанной оси.
 * Привод — гравитационный: наклоном `g` (setSlopeDeg) моделируется горка,
 * машина катится по курсу сама. Момента на колёсах нет — на этом dmech он
 * не передаёт движение раме.
 */
final class CarPhysics
{
    private PhysicsWorld world;

    /// Тело земли: статичный бокс, верхняя грань на z == 0.
    private RigidBody ground;

    /// Тело рамы: массы всех балок, инерция от AABB каркаса.
    private RigidBody chassis;

    /// Тела колёс по индексам `Frame.anchors`.
    private RigidBody[] wheelBodies;

    this(const Frame frame, const vec3 offset)
    {
        world = New!PhysicsWorld(null, 1000);
        world.gravity = Vector3f(0.0f, 0.0f, -9.80665f); // Z вверх

        auto g = world.addStaticBody(Vector3f(0.0f, 0.0f, -0.5f));
        g.friction = 0.9f;
        ground = g;
        // GeomBox берёт ПОЛОВИННЫЕ габариты: размер 0.5 в Z + тело в z=-0.5
        // даёт верхнюю грань ровно на z == 0.
        world.addShapeComponent(g, New!GeomBox(world, Vector3f(60.0f, 60.0f, 0.5f)),
            Vector3f(0.0f, 0.0f, 0.0f), 1.0f);

        buildChassis(frame, offset);
        buildWheels(frame, offset);
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

    private void buildChassis(const Frame frame, const vec3 offset)
    {
        // Масса и центр масс по балкам (каждая — цилиндр с осью вдоль длины).
        float totalMass = 0.0f;
        vec3 sumM = vec3(0.0f);
        foreach (b; frame.beams)
        {
            const vec3 a = frame.nodes[b.a].pos + offset;
            const vec3 b2 = frame.nodes[b.b].pos + offset;
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
            const vec3 p = n.pos + offset;
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

    private void buildWheels(const Frame frame, const vec3 offset)
    {
        wheelBodies.length = frame.anchors.length;

        foreach (i, a; frame.anchors)
        {
            const vec3 nodePos = frame.nodes[a.node].pos + offset;

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