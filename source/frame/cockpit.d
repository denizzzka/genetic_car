/**
 * Водительская кабина: масса, момент инерции и геометрия посадки.
 *
 * Меш кабины вшивается в бинарник на этапе компиляции (строковый
 * `import`), а разбирается в рантайме штатным читателем dagon
 * через `frame.objmesh` — свой парсер не пишем.
 *
 * Из меша вычисляется всё, что нужно физике и генетике:
 *  - точка опоры (низ кабины) — пересечение луча из центра масс (0,0,0
 *    координат OBJ, задан там же) вниз с мешем; она же seed-узел каркаса;
 *  - высота центра масс над этой точкой (для привязки к узлу 0).
 *
 * Оси координат OBJ-файла — базис каркаса (right = +X, forward = −Y,
 * up = +Z); единицы — миллиметры, перевод в метры делает
 * `ObjLoadOptions.scale`.
 */
module frame.cockpit;

import std.math : abs;

import dlib.core.memory;
import dlib.math.vector;

import frame.objmesh : ObjModel, loadObjText;

/// Текст меша кабины, вшитый в бинарник компилятором (нужен
/// `stringImportPaths "assets"` в dub.sdl).
enum cockpitObjText = import("driver_seat_boundary.obj");

/// Масса кабины, кг.
enum float cockpitMass = 100.0f;

/// Диагональ момента инерции кабины в осях OBJ-файла (Ixx, Iyy, Izz), кг·м².
/// Задана дизайном; соответствует setMassSpaceInertiaTensor(16.7, 16.7, 8.5).
enum vec3 cockpitInertia = vec3(16.7f, 16.7f, 8.5f);

/// Геометрия кабины, вычисленная из меша.
struct CockpitGeometry
{
    /// Точка опоры (низ кабины): пересечение луча из центра масс вниз с
    /// мешем. В координатах каркаса (м). Служит seed-узлом каркаса.
    vec3 seed;

    /// Высота центра масс кабины над точкой опоры (= −seed.z), м.
    float cogHeight;

    /// Габарит кабины по осям (max −min), м.
    vec3 dims;
}

/// Загрузить меш кабины из вшитого текста (парсинг в рантайме).
ObjModel loadCockpit()
{
    return loadObjText(cockpitObjText);
}

/// Геометрия кабины из меша: точка опоры и высота центра масс.
/// Центр масс считается в (0,0,0) координат OBJ — луч идёт оттуда вниз.
CockpitGeometry cockpitGeometry(const ObjModel model)
{
    CockpitGeometry g;

    vec3 minP = vec3(float.max), maxP = vec3(-float.max);
    foreach (v; model.mesh.vertices)
    {
        minP.x = minP.x < v.x ? minP.x : v.x;
        minP.y = minP.y < v.y ? minP.y : v.y;
        minP.z = minP.z < v.z ? minP.z : v.z;
        maxP.x = maxP.x > v.x ? maxP.x : v.x;
        maxP.y = maxP.y > v.y ? maxP.y : v.y;
        maxP.z = maxP.z > v.z ? maxP.z : v.z;
    }
    g.dims = maxP - minP;

    // Луч из центра масс (0,0,0) вниз (−up): ближайшее пересечение с мешем.
    const vec3 origin = vec3(0.0f, 0.0f, 0.0f);
    const vec3 dir = vec3(0.0f, 0.0f, -1.0f);
    float bestT = float.max;
    vec3 best = vec3(0.0f, 0.0f, 0.0f);
    bool hit = false;
    foreach (i; 0 .. model.mesh.indices.length)
    {
        const vec3 v0 = model.mesh.vertices[model.mesh.indices[i][0]];
        const vec3 v1 = model.mesh.vertices[model.mesh.indices[i][1]];
        const vec3 v2 = model.mesh.vertices[model.mesh.indices[i][2]];
        const float t = rayTriangle(origin, dir, v0, v1, v2);
        if (t > 0.0f && t < bestT)
        {
            bestT = t;
            best = origin + dir * t;
            hit = true;
        }
    }
    assert(hit, "кабина: луч из центра масс не пересёк меш");
    g.seed = best;
    g.cogHeight = -best.z;
    assert(g.cogHeight > 0.0f, "кабина: центр масс не выше точки опоры");
    return g;
}

/// Пересечение луча с треугольником (Мёллер–Трумбор): параметр t > 0
/// или отрицательное значение, если не пересекается.
private float rayTriangle(const vec3 o, const vec3 d,
    const vec3 v0, const vec3 v1, const vec3 v2)
{
    const vec3 e1 = v1 - v0;
    const vec3 e2 = v2 - v0;
    const vec3 p = cross(d, e2);
    const float det = dot(e1, p);
    if (abs(det) < 1e-12f)
        return -1.0f;
    const float invDet = 1.0f / det;
    const vec3 t = o - v0;
    const float u = dot(t, p) * invDet;
    if (u < 0.0f || u > 1.0f)
        return -1.0f;
    const vec3 q = cross(t, e1);
    const float v = dot(d, q) * invDet;
    if (v < 0.0f || u + v > 1.0f)
        return -1.0f;
    return dot(e2, q) * invDet;
}

unittest
{
    auto model = loadCockpit();
    scope (exit) Delete(model.asset);

    const g = cockpitGeometry(model);

    // Меш разобран:20 треугольников, вершины в метрах.
    assert(model.mesh.indices.length == 20, "20 треугольников корпуса");

    // Точка опоры — низ корпуса: луч вниз из (0,0,0) бьёт в нижнюю грань.
    assert(g.seed.z < 0.0f, "точка опоры ниже центра масс");
    assert(abs(g.seed.x) < 1e-4f && abs(g.seed.y) < 1e-4f,
        "луч вертикальный — точка опоры под центром масс");

    // Высота центра масс — по модулю чуть больше половины корпуса.
    assert(g.cogHeight > 0.1f && g.cogHeight < 3.0f,
        "высота центра масс в разумных пределах");

    // Габарит — корпус кабины, а не что-то масштаба километра.
    assert(g.dims.x > 0.0f && g.dims.x < 3.0f);
    assert(g.dims.y > 0.0f && g.dims.y < 3.0f);
    assert(g.dims.z > 0.0f && g.dims.z < 3.0f);

    // Константы дизайна на месте.
    assert(cockpitMass == 100.0f);
    assert(cockpitInertia.x == 16.7f && cockpitInertia.y == 16.7f
        && cockpitInertia.z == 8.5f);
}
