/**
 * Загрузка границы зоны сиденья водителя из Wavefront OBJ.
 *
 * Парсинг делегируется встроенному в dagon читателю `dagon.resource.obj.OBJAsset` —
 * никто не пишет свой парсер. Вызывается только его потокобезопасная часть:
 * `loadThreadSafePart` использует лишь имя файла (для предупреждений) и входной
 * поток, параметры `fs`/`mngr` в её теле не задействованы. GL не нужен —
 * `prepareVAO()` вьюер вызывает сам в GL-контексте.
 *
 * На выходе `ObjModel`:
 *  - рендер — `model.mesh` (dagon `Mesh`, данные готовы, VAO готовит вьюер),
 *    сущность вешается под carRoot;
 *  - физика — меш сам является `TriangleSet`, его напрямую потребляет
 *    `newBoundaryBody(model, world)`: статическое дерево-коллизия Newton,
 *    развёрнутое тем же поворотом «каркас → мир», что и остальные тела мира,
 *    поэтому ловит машину ровно там, где её видит рендер.
 *
 * Координаты файла читаются в базисе каркаса машины (right = +X, forward = −Y,
 * up = +Z) и переводятся в метры через `ObjLoadOptions.scale` (исходник в мм).
 *
 * Материалы и текстуры OBJ не читаются; нормали генерируются при отсутствии vn.
 * Из ограничений читателя dagon: без отрицательных индексов и N-угольников —
 * только треугольники и четырёхугольники. Вершины не дедуплицируются
 * (3 вершины на треугольник).
 */
module frame.objmesh;

import std.exception : enforce;
import std.file : exists, read;

import dlib.core.memory;
import dlib.core.stream;
import dlib.math.quaternion;
import dlib.math.vector;

import dagon.resource.obj;
import dagon.graphics.mesh;
import dagon.ext.newton;

import frame.frame : origin, up, right, forward;
import physics_world.physics : ensureNewtonLoaded, newtonBodyMatrix, PhysicsWorld;

/// Параметры интерпретации OBJ-файла.
struct ObjLoadOptions
{
    /// Масштаб координат файла в метры. Проект хранит границы в миллиметрах.
    float scale = 0.001f;

    /// Направления каркаса для осей OBJ (x, y, z); по умолчанию идентичность.
    vec3 axisX = vec3(1.0f, 0.0f, 0.0f);
    vec3 axisY = vec3(0.0f, 1.0f, 0.0f);
    vec3 axisZ = vec3(0.0f, 0.0f, 1.0f);
}

/// Результат загрузки: меш dagon, готовый и к рендеру, и к физике.
struct ObjModel
{
    /// Владелец меша; держит его живым. Перед выгрузкой `Delete(model.asset)` —
    /// это освободит и `mesh`.
    OBJAsset asset;

    /// Меш в координатах каркаса, метры (синоним `asset.mesh`). `dataReady`
    /// уже установлен; `prepareVAO()` вьюер делает сам в GL-контексте.
    /// Реализует `TriangleSet` — можно кормить `NewtonMeshShape` напрямую.
    Mesh mesh;
}

/// Загрузить OBJ из файла.
ObjModel loadObjMesh(string filename, const ObjLoadOptions options = ObjLoadOptions.init)
{
    enforce(exists(filename), "не найден файл: " ~ filename);
    return buildObjModel(cast(ubyte[]) read(filename), filename, options);
}

/// Загрузить OBJ из текста (удобно для тестов и встраиваемых данных).
ObjModel loadObjText(string text, const ObjLoadOptions options = ObjLoadOptions.init)
{
    return buildObjModel(cast(ubyte[]) text, "<текст>", options);
}

/// Общая сборка: парсинг встроенным читателем, перенос осей OBJ на базис
/// каркаса и перевод координат в метры.
private ObjModel buildObjModel(ubyte[] data, string filename,
    const ObjLoadOptions options)
{
    auto stream = New!ArrayStream(data);
    scope (exit) Delete(stream);

    auto asset = New!OBJAsset(null);
    enforce(asset.loadThreadSafePart(filename, stream, null, null),
        "не удалось разобрать «" ~ filename ~ "»");

    const vec3 ax = options.axisX;
    const vec3 ay = options.axisY;
    const vec3 az = options.axisZ;
    foreach (ref v; asset.mesh.vertices)
    {
        const vec3 src = v;
        v = src.x * ax + src.y * ay + src.z * az;
        v *= options.scale;
    }
    // Базис каркаса ортонормирован — нормали поворачиваются как вершины.
    foreach (ref n; asset.mesh.normals)
    {
        const vec3 src = n;
        n = src.x * ax + src.y * ay + src.z * az;
        n.normalize();
    }
    asset.mesh.calcBoundingBox();

    ObjModel model;
    model.asset = asset;
    model.mesh = asset.mesh;
    return model;
}

/// Tree-коллизия Newton из меша каркаса. Вершины остаются в координатах
/// каркаса — ориентацию в мир несёт тело.
NewtonMeshShape newtonTreeShape(Mesh mesh, NewtonPhysicsWorld world)
{
    ensureNewtonLoaded();
    return New!NewtonMeshShape(mesh, world);
}

/// Статическое тело-граница в мире Newton: та же геометрия, что у
/// `model.mesh`, развёрнутая поворотом «каркас → мир» и стоящая в origin
/// (точка старта машины). Коллизия ловит машину ровно там, где рендер меша
/// под carRoot.
NewtonRigidBody newBoundaryBody(ObjModel model, NewtonPhysicsWorld world)
{
    auto shape = newtonTreeShape(model.mesh, world);
    auto body = New!NewtonRigidBody(NewtonRigidBodyType.Static, shape, 0.0f,
        world, world);
    body.dynamic = false;
    body.setTransformation(newtonBodyMatrix(origin, Quaternionf.identity));
    body.update(0.0);
    return body;
}

unittest
{
    import std.math : abs;

    // Зоопарк форматов чтения граней: квад v/t/n, треугольник v//n,
    // треугольник v, квад v — вся палитра поддерживаемых вариантов.
    const txt = `# test
v -1 -1 0
v 1 -1 0
v 1 1 0
v -1 1 0
v -1 -1 1
v 1 -1 1
v 1 1 1
v -1 1 1
vn 0 0 1
vt 0 0
vt 1 0
vt 0 1
vt 1 1
f 1/1/1 2/2/1 3/3/1 4/1/1
f 5//1 6//1 7//1
f 8 7 6
f 1 2 3 4
`;
    ObjLoadOptions opt;
    opt.scale = 1.0f;
    auto model = loadObjText(txt, opt);
    scope (exit) Delete(model.asset);

    assert(model.mesh !is null);
    assert(model.mesh.indices.length == 6, "2 три от квадов + 2 три");
    assert(model.mesh.vertices.length == 18, "по 3 вершины на треугольник");
    assert(model.mesh.vertices[0].x == -1.0f && model.mesh.vertices[0].y == -1.0f);

    foreach (ref idx; model.mesh.indices)
        foreach (i; idx)
            assert(i < model.mesh.vertices.length, "индекс меша вне диапазона");

    // Нормали пришли из vn.
    foreach (n; model.mesh.normals)
        assert(abs(n.length - 1.0f) < 1e-4f, "нормаль не единичная");
}

unittest
{
    import std.math : abs;

    // Без vn — нормали должны сгенерироваться.
    const txt = `# плоскость без нормалей и текстур
v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
f 1 2 3
f 1 3 4
`;
    ObjLoadOptions opt;
    opt.scale = 1.0f;
    auto model = loadObjText(txt, opt);
    scope (exit) Delete(model.asset);

    assert(model.mesh.indices.length == 2);
    foreach (n; model.mesh.normals)
        assert(abs(n.length - 1.0f) < 1e-4f, "нормали сгенерированы единичные");
}

unittest
{
    import std.math : abs;

    if (!exists("assets/driver_seat_boundary.obj"))
        return;

    // Отображение осей OBJ на каркас, как в loadCockpit.
    ObjLoadOptions opt;
    opt.axisX = right;
    opt.axisY = up;
    opt.axisZ = forward;
    auto model = loadObjMesh("assets/driver_seat_boundary.obj", opt);
    scope (exit) Delete(model.asset);

    assert(model.mesh.indices.length == 20, "20 треугольников корпуса");
    assert(model.mesh.vertices.length == 60, "по 3 вершины на треугольник");

    // Масштаб мм → м и Axes: первая вершина (-350, -449, -695) мм
    // даёт каркасные (-0.35, +0.695, -0.449).
    assert(abs(model.mesh.vertices[0].x - (-0.35f)) < 1e-5f);
    assert(abs(model.mesh.vertices[0].y - 0.695f) < 1e-5f);
    assert(abs(model.mesh.vertices[0].z - (-0.449f)) < 1e-5f);

    foreach (ref idx; model.mesh.indices)
        foreach (i; idx)
            assert(i < model.mesh.vertices.length, "индекс меша вне диапазона");

    // Корпус в метрах: ±3 м по осям.
    foreach (v; model.mesh.vertices)
        assert(abs(v.x) <= 3.0f && abs(v.y) <= 3.0f && abs(v.z) <= 3.0f,
            "вершина вне ожидаемого корпуса");

    foreach (n; model.mesh.normals)
        assert(abs(n.length - 1.0f) < 1e-4f, "нормали сгенерированы единичные");
}

unittest
{
    import dagon.core.event : EventManager;
    import dlib.core.ownership : Owner;

    if (!exists("assets/driver_seat_boundary.obj"))
        return;

    ensureNewtonLoaded();
    auto model = loadObjMesh("assets/driver_seat_boundary.obj");
    scope (exit) Delete(model.asset);

    auto world = New!PhysicsWorld(cast(EventManager) null, cast(Owner) null);
    scope (exit) Delete(world);

    auto body = newBoundaryBody(model, world);
    assert(body.newtonBody !is null, "граница создала тело в Newton");
    assert(body.dynamic == false, "граница статична");
}