module viewer.wheelmesh;

import std.math : PI, cos, sin;

import dlib.core.ownership;
import dlib.math.vector;

import dagon;

import physics_world.engine : tubeSegments;
import physics_world.physics : wheelInnerRadius, wheelRadius, wheelWidth;

/// Покрышка колеса в габаритах физической трубы `tubeShape`: внешний радиус
/// `outer`, внутренний `inner`, ширина `height`, ось вдоль локальной Y.
/// Встаёт на место тора: у того круглое сечение, а катит труба — с плоским
/// протектором, и на земле рендер стоял выше коллизии.
Mesh buildWheelMesh(Owner owner, const float outer = wheelRadius,
    const float inner = wheelInnerRadius, const float height = wheelWidth)
{
    enum size_t faceQuads = 4 * tubeSegments;

    auto mesh = New!Mesh(owner);
    mesh.vertices = New!(Vector3f[])(faceQuads * 4);
    mesh.normals = New!(Vector3f[])(faceQuads * 4);
    mesh.texcoords = New!(Vector2f[])(faceQuads * 4);
    mesh.indices = New!(uint[3][])(faceQuads * 2);

    const float hh = 0.5f * height;
    // Грань трубы стоит на расстоянии радиуса от оси, поэтому вершины
    // многоугольника берём дальше: r / cos(π/N). Иначе рендер уже физики.
    const float circum = 1.0f / cos(PI / tubeSegments);
    const float rOut = outer * circum;
    const float rIn = inner * circum;

    size_t vi;
    size_t ii;
    foreach (k; 0 .. tubeSegments)
    {
        const float u0 = cast(float) k / cast(float) tubeSegments;
        const float u1 = cast(float)(k + 1) / cast(float) tubeSegments;
        const float a0 = 2.0f * PI * u0;
        const float a1 = 2.0f * PI * u1;
        const float amid = 0.5f * (a0 + a1);
        const Vector3f outMid = Vector3f(cos(amid), 0.0f, sin(amid));

        // Внешняя стенка: порядок вершин выводит нормаль наружу, как у трубы.
        pushQuad(mesh, vi, ii, outMid,
            [ring(rOut, a0, -hh), ring(rOut, a0, hh),
             ring(rOut, a1, hh), ring(rOut, a1, -hh)],
            [Vector2f(u0, 0.0f), Vector2f(u0, 1.0f),
             Vector2f(u1, 1.0f), Vector2f(u1, 0.0f)]);

        // Внутренняя стенка — наоборот: её нормаль смотрит в отверстие.
        pushQuad(mesh, vi, ii, -outMid,
            [ring(rIn, a1, -hh), ring(rIn, a1, hh),
             ring(rIn, a0, hh), ring(rIn, a0, -hh)],
            [Vector2f(u1, 0.0f), Vector2f(u1, 1.0f),
             Vector2f(u0, 1.0f), Vector2f(u0, 0.0f)]);

        // Днища: v идёт по радиусу от отверстия к ободу.
        pushQuad(mesh, vi, ii, Vector3f(0.0f, -1.0f, 0.0f),
            [ring(rIn, a0, -hh), ring(rOut, a0, -hh),
             ring(rOut, a1, -hh), ring(rIn, a1, -hh)],
            [Vector2f(u0, 0.0f), Vector2f(u0, 1.0f),
             Vector2f(u1, 1.0f), Vector2f(u1, 0.0f)]);

        pushQuad(mesh, vi, ii, Vector3f(0.0f, 1.0f, 0.0f),
            [ring(rOut, a0, hh), ring(rIn, a0, hh),
             ring(rIn, a1, hh), ring(rOut, a1, hh)],
            [Vector2f(u0, 1.0f), Vector2f(u0, 0.0f),
             Vector2f(u1, 0.0f), Vector2f(u1, 1.0f)]);
    }

    mesh.dataReady = true;
    mesh.calcBoundingBox();
    mesh.prepareVAO();
    return mesh;
}

/// Точка кольца радиуса `r` под углом `angle`, на высоте `y` (ось колеса — Y).
private vec3 ring(const float r, const float angle, const float y)
{
    return vec3(r * cos(angle), y, r * sin(angle));
}

/// Грань с плоской нормалью: четыре свои вершины, два треугольника.
private void pushQuad(Mesh mesh, ref size_t vi, ref size_t ii,
    const Vector3f n, const vec3[4] p, const Vector2f[4] t)
{
    foreach (i; 0 .. 4)
    {
        mesh.vertices[vi + i] = p[i];
        mesh.normals[vi + i] = n;
        mesh.texcoords[vi + i] = t[i];
    }
    mesh.indices[ii] = [cast(uint) vi, cast(uint)(vi + 1), cast(uint)(vi + 2)];
    mesh.indices[ii + 1] = [cast(uint) vi, cast(uint)(vi + 2), cast(uint)(vi + 3)];
    vi += 4;
    ii += 2;
}