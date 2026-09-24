/**
 * Водительская кабина: масса, момент инерции и геометрия посадки.
 *
 * Меш кабины вшивается в бинарник на этапе компиляции (строковый
 * `import`), а разбирается в рантайме штатным читателем dagon
 * через `frame.objmesh` — свой парсер не пишем.
 *
 * Крепление к каркасу — через центр масс: узел 0 каркаса совмещён с ЦМ
 * кабины (0,0,0 координат OBJ). Под корпусом на общей плоскости висит
 * «хребет» — станции под днищем (x = 0) с боковыми парами в станциях, где
 * рёбра днища параллельны оси ширины; они служат точками крепления балок.
 *
 * Оси координат OBJ-файла — базис каркаса (right = +X, forward = −Y,
 * up = +Z); единицы — миллиметры, перевод в метры делает
 * `ObjLoadOptions.scale`.
 */
module frame.cockpit;

import std.math : abs;
import std.algorithm : sort;

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

/// Просвет хребта: на сколько ниже дна корпуса висят точки крепления.
enum float keelClear = 0.05f;

/// Геометрия кабины, вычисленная из меша.
struct CockpitGeometry
{
    /// Габарит кабины по осям (max − min), м.
    vec3 dims;

    /// Углы нижней грани корпуса, относительно ЦМ (node0). Касание земли
    /// любой точкой этой грани — сход.
    vec3[4] floorCorners;

    /// Нижний и верхний углы корпуса, относительно ЦМ; через них крепление
    /// каркаса не проходит (запретная зона в fitness).
    vec3 minP, maxP;

    /// «Хребет»: станции крепления под днищем, общая плоскость keelZ.
    vec3[] spine;

    /// Индексы станций хребта с боковыми точками (spineSideAt[i] → spine).
    size_t[] spineSideAt;

    /// Боковые пары (right, left) станций крепления.
    vec3[2][] spineSides;
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
// вычисляется из меша один раз при старте программы (static this) и дальше
// не меняется; массивы внутри просто не трогаются после инициализации.
private static CockpitGeometry cockpitGeom_;

static this()
{
    auto model = loadCockpit();
    scope (exit) Delete(model.asset);
    cockpitGeom_ = cockpitGeometry(model);
}

/// Геометрия кабины (кэш: вычислена из меша на старте программы).
// TODO: кэш должен быть static immutable (вернёт настоящую immutable копию), но
// сейчас структура с динамическими массивами не даёт вернуть её по значению.
const(CockpitGeometry) cockpitGeometry()
{
    return cockpitGeom_;
}

/// Геометрия кабины из меша: AABB, углы дна и хребет крепления.
/// Центр масс — в (0,0,0) координат OBJ, он же узел 0 каркаса.
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
    g.minP = minP;
    g.maxP = maxP;

    // Углы нижней грани — в координатах центра масс (плоское дно).
    foreach (ix; 0 .. 2)
        foreach (iy; 0 .. 2)
        {
            const float px = ix ? maxP.x : minP.x;
            const float py = iy ? maxP.y : minP.y;
            g.floorCorners[ix * 2 + iy] = vec3(px, py, minP.z);
        }

    buildKeel(g, model);
    return g;
}

/// Станции хребта: рёбра днища, пересекающие плоскость x = 0. Рёбра,
/// параллельные оси ширины, дают боковые пары с полушириной корпуса.
private void buildKeel(ref CockpitGeometry g, const ObjModel model)
{
    enum float triEps = 1e-5f;

    struct Station { float y; bool side; float half; }

    vec3[2][] seen;
    Station[] stations;

    foreach (i; 0 .. model.mesh.indices.length)
    {
        const auto vi = model.mesh.indices[i];
        const vec3 p0 = model.mesh.vertices[vi[0]];
        const vec3 p1 = model.mesh.vertices[vi[1]];
        const vec3 p2 = model.mesh.vertices[vi[2]];
        vec3 nrm = cross(p1 - p0, p2 - p0);
        nrm.normalize();
        if (nrm.z < -0.5f)
        {
            const vec3[3] tri = [p0, p1, p2];
            foreach (k; 0 .. 3)
            {
                const vec3 a = tri[k], b = tri[(k + 1) % 3];
                if (edgeSeen(seen, a, b))
                    continue;
                if (a.x * b.x > 0.0f)
                    continue; // ребро целиком с одной стороны x = 0
                const float t = a.x / (a.x - b.x);
                const vec3 ip = a + (b - a) * t;
                const bool xPar = abs(a.y - b.y) < triEps && abs(a.z - b.z) < triEps;
                stations ~= Station(ip.y, xPar, xPar ? abs(a.x) : 0.0f);
            }
        }
    }

    sort!"a.y < b.y"(stations);
    if (stations.length == 0)
        return;

    const keelZ = g.minP.z - keelClear;
    foreach (s; stations)
        g.spine ~= vec3(0.0f, s.y, keelZ);
    foreach (i; 0 .. stations.length)
        if (stations[i].side)
        {
            g.spineSideAt ~= i;
            const float h = stations[i].half;
            g.spineSides ~= [vec3(h, stations[i].y, keelZ),
                vec3(-h, stations[i].y, keelZ)];
        }
}

private bool edgeSeen(ref vec3[2][] seen, vec3 a, vec3 b)
{
    foreach (e; seen)
        if ((same3(e[0], a) && same3(e[1], b)) || (same3(e[0], b) && same3(e[1], a)))
            return true;
    const bool aL = less(a, b);
    seen ~= [aL ? a : b, aL ? b : a];
    return false;
}

private bool same3(const vec3 a, const vec3 b)
{
    return abs(a.x - b.x) < 1e-5f && abs(a.y - b.y) < 1e-5f
        && abs(a.z - b.z) < 1e-5f;
}

private bool less(const vec3 a, const vec3 b)
{
    return a.x < b.x || (a.x == b.x && (a.y < b.y || (a.y == b.y && a.z < b.z)));
}

unittest
{
    auto model = loadCockpit();
    scope (exit) Delete(model.asset);

    const g = cockpitGeometry(model);

    // Меш разобран:20 треугольников, вершины в метрах.
    assert(model.mesh.indices.length == 20, "20 треугольников корпуса");

    // Габарит — корпус кабины, а не что-то масштаба километра.
    assert(g.dims.x > 0.0f && g.dims.x < 3.0f);
    assert(g.dims.y > 0.0f && g.dims.y < 3.0f);
    assert(g.dims.z > 0.0f && g.dims.z < 3.0f);

    // Нижняя грань — плоское дно, в координатах центра масс; раскрыв по
    // габаритам.
    const float floorZ = g.floorCorners[0].z;
    foreach (c; g.floorCorners)
    {
        assert(c.z <= 0.0f, "угол пола не выше центра масс");
        assert(abs(c.z - floorZ) < 1e-4f, "углы пола в одной плоскости");
    }
    float xlo = float.max, xhi = -float.max, ylo = float.max, yhi = -float.max;
    foreach (c; g.floorCorners)
    {
        xlo = xlo < c.x ? xlo : c.x; xhi = xhi > c.x ? xhi : c.x;
        ylo = ylo < c.y ? ylo : c.y; yhi = yhi > c.y ? yhi : c.y;
    }
    assert(xlo == g.minP.x && xhi == g.maxP.x, "углы пола на границе габарита x");
    assert(ylo == g.minP.y && yhi == g.maxP.y, "углы пола на границе габарита y");

    // Хребет: станции по центру, общая плоскость ниже дна корпуса.
    assert(g.spine.length == 5, "пять станций под днищем");
    foreach (s; g.spine)
    {
        assert(abs(s.x) < 1e-4f, "станции по оси ширины виляют");
        assert(abs(s.z - (g.minP.z - keelClear)) < 1e-4f,
            "станции на общей плоскости ниже дна");
    }
    // Станции упорядочены от носа к корме, без повторов y.
    float prevY = -float.max;
    foreach (s; g.spine)
    {
        assert(s.y > prevY, "станции идут от носа к корме");
        prevY = s.y;
    }

    // Боковые пары: в станциях, где рёбра днища параллельны ширине.
    assert(g.spineSideAt == [0, 2, 4], "боковые пары на параллельных рёбрах");
    assert(g.spineSides.length == g.spineSideAt.length);
    foreach (i; 0 .. g.spineSides.length)
    {
        const vec3 right = g.spineSides[i][0], left = g.spineSides[i][1];
        const size_t sIdx = g.spineSideAt[i];
        assert(right.x > 0.0f && right.x == -left.x, "пара зеркальна");
        assert(abs(right.y - g.spine[sIdx].y) < 1e-4f, "пара в станции");
        const vec3 half = g.maxP;
        assert(right.x <= half.x && right.x > 0.0f, "пара в раскрыве корпуса");
    }

    // Константы дизайна на месте.
    assert(cockpitMass == 100.0f);
    assert(cockpitInertia.x == 16.7f && cockpitInertia.y == 8.5f
        && cockpitInertia.z == 16.7f);
}
