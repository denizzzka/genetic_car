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

import frame.frame : right, up, forward;
import frame.objmesh : ObjModel, loadObjText, ObjLoadOptions;

/// Текст меша кабины, вшитый в бинарник компилятором (нужен
/// `stringImportPaths "assets"` в dub.sdl).
enum cockpitObjText = import("driver_seat_boundary.obj");

/// Масса кабины, кг.
enum float cockpitMass = 100.0f;

/// Диагональ момента инерции кабины в осях каркаса (Ixx, Iyy, Izz), кг·м².
/// Задана дизайном в осях OBJ (16.7, 16.7, 8.5); 8.5 — вокруг длинной оси
/// (перед-зад), в каркасе это ось y.
enum vec3 cockpitInertia = vec3(16.7f, 8.5f, 16.7f);

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

    /// Углы нижней грани корпуса относительно точки опоры (координаты
    /// каркаса): касание земли любой точкой этой грани — сход. Одной точки
    /// опоры при крене не хватает: край корпуса упирается в грунт раньше
    /// вертикали под центром масс.
    vec3[4] floorCorners;
}

/// Загрузить меш кабины из вшитого текста (парсинг в рантайме).
ObjModel loadCockpit()
{
    // Оси OBJ в каркасе: x — right, y — up, z — forward (длина, перед-зад).
    ObjLoadOptions opt;
    opt.axisX = right;
    opt.axisY = up;
    opt.axisZ = forward;
    return loadObjText(cockpitObjText, opt);
}

// Геометрия кабины — константа дизайна: меш вшит в бинарник, поэтому она
// вычисляется из меша один раз при старте программы (shared static this)
// и дальше живёт как immutable — мьютекс не нужен даже на пуле воркеров.
private static immutable CockpitGeometry cockpitGeom_;

shared static this()
{
    auto model = loadCockpit();
    scope (exit) Delete(model.asset);
    cockpitGeom_ = cockpitGeometry(model);
}

/// Геометрия кабины (кэш: вычислена из меша на старте программы).
CockpitGeometry cockpitGeometry()
{
    return cockpitGeom_;
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

    // Углы нижней грани — от минимума по вертикали (плоское дно) и раскрыва
    // по габаритам; от центра масс отнимается точка опоры, чтобы углы были
    // в координатах места крепления (низ на узле 0).
    foreach (ix; 0 .. 2)
        foreach (iy; 0 .. 2)
        {
            const float px = ix ? maxP.x : minP.x;
            const float py = iy ? maxP.y : minP.y;
            g.floorCorners[ix * 2 + iy] =
                vec3(px - best.x, py - best.y, minP.z - best.z);
        }
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

    // Нижняя грань — плоское дно (может лежать ниже точки опоры: луч из ЦМ
    // бьёт в приподнятый центр днища), раскрыв по габаритам.
    foreach (c; g.floorCorners)
        assert(c.z <= 0.0f, "угол пола не выше точки опоры");
    const float floorZ = g.floorCorners[0].z;
    foreach (c; g.floorCorners)
        assert(abs(c.z - floorZ) < 1e-4f, "углы пола в одной плоскости");
    float xlo = float.max, xhi = -float.max, ylo = float.max, yhi = -float.max;
    foreach (c; g.floorCorners)
    {
        xlo = xlo < c.x ? xlo : c.x; xhi = xhi > c.x ? xhi : c.x;
        ylo = ylo < c.y ? ylo : c.y; yhi = yhi > c.y ? yhi : c.y;
    }
    assert(abs(xhi - xlo - g.dims.x) < 1e-4f, "углы раскрывают габарит по ширине");
    assert(abs(yhi - ylo - g.dims.y) < 1e-4f, "углы раскрывают габарит по длине");

    // Константы дизайна на месте.
    assert(cockpitMass == 100.0f);
    assert(cockpitInertia.x == 16.7f && cockpitInertia.y == 8.5f
        && cockpitInertia.z == 16.7f);
}
