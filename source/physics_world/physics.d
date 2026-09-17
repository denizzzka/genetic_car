module physics_world.physics;

import std.math;

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

    // Простейший багги: колесо, балка и моторное колесо на другом конце.
    Frame frame;
    size_t node(vec3 pos)
    {
        frame.nodes ~= Node(pos);
        return frame.nodes.length - 1;
    }

    const wheel = node(vec3(0.6f, 0.7f, 0.3f));
    const motor = node(vec3(0.6f, -0.7f, 0.3f));
    const top = node(vec3(0.6f, 0.0f, 0.9f));
    frame.beams ~= Beam(wheel, motor, 0.05f);
    frame.beams ~= Beam(wheel, top, 0.03f);
    frame.anchors ~= Anchor(wheel, AnchorKind.wheel);
    frame.anchors ~= Anchor(motor, AnchorKind.motorWheel);

    // Подъём по Z, как в viewer: низ самого низкого колеса на z == 0.
    vec3 offset = vec3(0.0f);
    float minZ = float.max;
    foreach (a; frame.anchors)
        if (frame.nodes[a.node].pos.z < minZ)
            minZ = frame.nodes[a.node].pos.z;
    offset.z = wheelRadius - minZ;

    auto physics = new CarPhysics(frame, offset);
    scope (exit) physics.dispose();

    // ~10 секунд симуляции с газом — машина должна остаться на земле,
    // не разлететься и не провалиться сквозь неё.
    const double dt = 1.0 / 60.0;
    foreach (i; 0 .. 600)
        physics.step(dt, 0.5f);

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
            "позиция балки не конечна — машина разлетелась");
    }
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

/// Плотность материала балок каркаса, кг/м^3.
enum float beamDensity = 150.0f;

/// Крутящий момент ведущего колеса, Н·м.
enum float motorTorque = 35.0f;

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
 * Каркас: каждая балка — отдельное RigidBody, узлы — соединения без тел
 * (попарные BallConstraint в узле). Колёса — тела у якорей, навешены на балку
 * шарниром (HingeConstraint) с осью X; вращение/привод — applyTorque.
 * Коллизии имеют только колёса (с землёй); балки стоят на связях, иначе
 * перекрытия в узлах выдавливали бы каркас.
 */
final class CarPhysics
{
    private PhysicsWorld world;

    /// Тело земли: статичный бокс, верхняя грань на z == 0.
    private RigidBody ground;

    /// Тела балок по индексам `Frame.beams` (null — вырожденная балка).
    private RigidBody[] beamBodies;

    /// Тела колёс по индексам `Frame.anchors` и признак ведущего колеса.
    private RigidBody[] wheelBodies;
    private bool[] wheelDrive;

    this(const Frame frame, const vec3 offset)
    {
        world = New!PhysicsWorld(null, 1000);
        world.gravity = Vector3f(0.0f, 0.0f, -9.80665f); // Z вверх

        auto g = world.addStaticBody(Vector3f(0.0f, 0.0f, -0.5f));
        g.friction = 0.9f;
        ground = g;
        world.addShapeComponent(g, New!GeomBox(world, Vector3f(5.0f, 5.0f, 0.5f)),
            Vector3f(0.0f, 0.0f, 0.0f), 1.0f);

        beamBodies.length = frame.beams.length;
        buildBeams(frame, offset);
        buildWheels(frame, offset);
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
        beamBodies.length = 0;
        wheelBodies.length = 0;
        wheelDrive.length = 0;
    }

    /// Один шаг симуляции. Фиксированный dt (~1/60) надёжно стабилен.
    /// `throttle` — газ по оси Y: -1..+1; применяется к ведущим колёсам до update.
    void step(double dt, float throttle)
    {
        if (world is null)
            return;

        if (throttle != 0.0f)
        {
            immutable Vector3f axis = Vector3f(1.0f, 0.0f, 0.0f);
            foreach (i, w; wheelBodies)
                if (wheelDrive[i])
                    w.applyTorque(axis * (motorTorque * throttle));
        }

        world.update(dt);
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

    private void buildBeams(const Frame frame, const vec3 offset)
    {
        foreach (i, b; frame.beams)
        {
            const vec3 a = frame.nodes[b.a].pos + offset;
            const vec3 b2 = frame.nodes[b.b].pos + offset;
            const vec3 dir = b2 - a;
            const float len = dir.length;
            if (len < 1e-5f)
                continue;

            auto body = world.addDynamicBody((a + b2) * 0.5f, 0.0f);
            body.orientation = rotationBetween(Vector3f(0, 1, 0), dir / len);
            body.stopThreshold = 0.0f;

            float radius = b.radius;
            float mass = cast(float)(beamDensity * PI * radius * radius * len);
            auto shape = world.addShapeComponent(body,
                New!GeomCylinder(world, len, radius),
                Vector3f(0.0f, 0.0f, 0.0f), mass);
            shape.active = false;

            beamBodies[i] = body;
        }

        // Узел: все инцидентные балки сведены в точке попарными BallConstraint.
        size_t[][] nodeBeams = new size_t[][](frame.nodes.length);
        foreach (i, b; frame.beams)
            if (beamBodies[i] !is null)
            {
                nodeBeams[b.a] ~= i;
                nodeBeams[b.b] ~= i;
            }

        foreach (node, inc; nodeBeams)
        {
            const vec3 nodePos = frame.nodes[node].pos + offset;
            foreach (m; 0 .. inc.length)
                foreach (n; m + 1 .. inc.length)
                {
                    auto body1 = beamBodies[inc[m]];
                    auto body2 = beamBodies[inc[n]];
                    Vector3f anchor1 =
                        body1.orientation.conj.rotate(nodePos - body1.position);
                    Vector3f anchor2 =
                        body2.orientation.conj.rotate(nodePos - body2.position);
                    world.addConstraint(New!BallConstraint(
                        world, body1, body2, anchor1, anchor2));
                }
        }
    }

    private void buildWheels(const Frame frame, const vec3 offset)
    {
        wheelBodies.length = frame.anchors.length;
        wheelDrive.length = frame.anchors.length;

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

            // Навешиваем колесо на первую балку, инцидентную узлу якоря,
            // шарниром: точка узла закреплена, вращение свободно вокруг X.
            foreach (j, b; frame.beams)
            {
                if (beamBodies[j] is null)
                    continue;
                if (b.a == a.node || b.b == a.node)
                {
                    auto cBody = beamBodies[j];
                    Vector3f anchor =
                        cBody.orientation.conj.rotate(nodePos - cBody.position);
                    world.addConstraint(New!HingeConstraint(world, cBody, wheel,
                        anchor, Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 0.0f, 0.0f)));
                    break;
                }
            }

            wheelBodies[i] = wheel;
            wheelDrive[i] = (a.kind == AnchorKind.motorWheel);
        }
    }
}