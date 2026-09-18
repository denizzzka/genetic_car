module physics_world.physics;

import std.math;

import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;
import dlib.math.utils;
import dlib.geometry.aabb;

import dmech;

/// Радиус колеса. Совпадает с визуальным тором: ShapeTorus(0.2f, 0.1f)
/// даёт внешний радиус 0.2 + 0.1 = 0.3.
enum float wheelRadius = 0.3f;

/// Внутренний радиус «отверстия» колеса (покрышка — полый цилиндр).
enum float wheelInnerRadius = 0.22f;

/// Ширина колеса (толщина покрышки, равна диаметру трубы тора).
enum float wheelWidth = 0.2f;

/// Плотность материала колеса, кг/м^3.
enum float wheelDensity = 400.0f;

/// Развод якорей сварки (BallConstraint). Связь строит нормаль от разницы
/// якорей (`dp.normalized`); при точно совпадающих якорях разница нулевая,
/// нормаль вырождается в нуль и связь молча ничего не делает. Микро-отступ
/// (всегда > 0) превращает сварочный узел в реально работающую связь.
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