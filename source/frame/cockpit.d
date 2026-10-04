/**
 * Водительская кабина: масса, момент инерции и геометрия посадки.
 *
 * Меш кабины вшивается в бинарник на этапе компиляции (строковый
 * `import`), а разбирается в рантайме штатным читателем dagon
 * через `frame.objmesh` — свой парсер не пишем.
 *
 * Крепление к каркасу — через центр масс: узел 0 каркаса совмещён с ЦМ
 * кабины (0,0,0 координат OBJ). На контуре днища (пересечения рёбер меша
 * с x = 0) лежит «хребет» — станции (x = 0) с боковыми парами в станциях,
 * где рёбра днища параллельны оси ширины; они служат точками крепления.
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

import frame.frame : right, up, forward, origin, Frame, Node, EphemeralBeam, FrameContext;
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
    /// Габарит кабины по осям (max − min), м.
    vec3 dims;

    /// Углы нижней грани корпуса, относительно ЦМ (node0). Касание земли
    /// любой точкой этой грани — сход.
    vec3[4] floorCorners;

    /// Нижний и верхний углы корпуса, относительно ЦМ; через них крепление
    /// каркаса не проходит (запретная зона в fitness).
    vec3 minP, maxP;

    vec3[] floorProfile;
    vec3[] mountStations;
    size_t[] mountPairStations;
    vec3[2][] mountPairs;
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

/// Геометрия кабины из меша: AABB, профиль дна и точки крепления.
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

    buildMounts(g, model);
    return g;
}

/// Точки крепления из рёбер днища, пересекающих плоскость x = 0.
private void buildMounts(ref CockpitGeometry g, const ObjModel model)
{
    enum float triEps = 1e-5f;

    struct Station { float y; float z; bool side; float half; }

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
                stations ~= Station(ip.y, ip.z, xPar, xPar ? abs(a.x) : 0.0f);
            }
        }
    }

    sort!"a.y < b.y"(stations);
    if (stations.length == 0)
        return;

    foreach (s; stations)
    {
        const vec3 p = vec3(0.0f, s.y, s.z);
        g.floorProfile ~= p;
        g.mountStations ~= p;
    }
    foreach (i; 0 .. stations.length)
        if (stations[i].side)
        {
            g.mountPairStations ~= i;
            const float h = stations[i].half;
            g.mountPairs ~= [vec3(h, stations[i].y, stations[i].z),
                vec3(-h, stations[i].y, stations[i].z)];
        }
}

FrameContext cockpitFrameContext()
{
    const auto cg = cockpitGeometry();
    FrameContext context;
    context.frame.nodes ~= Node(origin);
    context.twinOf ~= 0;
    context.inertNode = 0;

    const size_t station0 = context.frame.nodes.length;
    foreach (p; cg.mountStations)
    {
        context.frame.nodes ~= Node(p);
        context.twinOf ~= context.frame.nodes.length - 1;
    }

    const size_t pair0 = context.frame.nodes.length;
    foreach (i, pair; cg.mountPairs)
    {
        const size_t right = pair0 + i * 2;
        context.frame.nodes ~= Node(pair[0]);
        context.frame.nodes ~= Node(pair[1]);
        context.twinOf ~= right + 1;
        context.twinOf ~= right;
    }
    context.growthNode = context.frame.nodes.length - 1;

    auto eph = (size_t a, size_t b) {
        context.frame.beams ~= new EphemeralBeam(a, b);
    };

    size_t best = 0;
    float bestD = distance(origin, cg.mountStations[0]);
    foreach (i; 1 .. cg.mountStations.length)
    {
        const float d = distance(origin, cg.mountStations[i]);
        if (d < bestD)
        {
            bestD = d;
            best = i;
        }
    }
    eph(0, station0 + best);
    foreach (i; 0 .. cg.mountStations.length - 1)
        eph(station0 + i, station0 + i + 1);

    foreach (j; 0 .. cg.mountPairs.length)
    {
        const size_t right = pair0 + j * 2;
        eph(station0 + cg.mountPairStations[j], right);
        eph(station0 + cg.mountPairStations[j], right + 1);
    }
    return context;
}

/**
 * Узлы крепления по бортам: пары точек подвески, к которым организм
 * прирастает. Индексы — в `cockpitFrameContext().frame`.
 *
 * Именно они, а не станции хребта: хребет — это сам корпус кабины, а бортовые
 * точки подвески — то, чему машина цепляется к земле.
 */
size_t[] cockpitMountNodes()
{
    const auto cg = cockpitGeometry();
    const size_t pair0 = 1 + cg.mountStations.length;
    size_t[] idx;
    foreach (i; 0 .. cg.mountPairs.length)
    {
        idx ~= pair0 + i * 2;
        idx ~= pair0 + i * 2 + 1;
    }
    return idx;
}

size_t cockpitFrameNodeCount()
{
    const auto cg = cockpitGeometry();
    return 1 + cg.mountStations.length + cg.mountPairs.length * 2;
}

size_t cockpitFrameBeamCount()
{
    const auto cg = cockpitGeometry();
    return 1 + (cg.mountStations.length - 1) + cg.mountPairs.length * 2;
}

/*
 * Запретная зона кабины живёт здесь, а не в фитнесе: о ней спрашивают и
 * отбор (нельзя ли построить такую машину), и рост (куда вообще можно
 * выпустить новую балку). Зона — параллелепипед от узла 0 (ЦМ кабины) по
 * AABB меша, пол — по контуру днища.
 */

/**
 * Пересекает ли отрезок строгую внутренность зоны кабины. Балка в самой
 * плоскости пола зону не задевает — ловится только реальный проход сквозь
 * корпус.
 */
bool beamHitsCabin(const Frame f, const vec3 a, const vec3 b)
{
    const auto cg = cockpitGeometry();
    return beamPiercesCabin(a, b, f.nodes[0].pos + cg.minP,
        f.nodes[0].pos + cg.maxP, cg, f.nodes[0].pos);
}

/**
 * Касается ли колесо кабины: центр диска внутри корпуса, расширенного на
 * радиус колеса; низ корпуса — по контуру днища на курсе колеса.
 */
bool wheelHitsCabin(const Frame f, const vec3 p, float r)
{
    const auto cg = cockpitGeometry();
    return wheelPiercesCabin(p, r, f.nodes[0].pos + cg.minP,
        f.nodes[0].pos + cg.maxP, cg, f.nodes[0].pos);
}

/// Пол запретной зоны на курсе y (локальный, от ЦМ кабины): контур днища,
/// интерполяция профиля. За пределами станций — крайние значения контура.
private float cabinFloor(const CockpitGeometry cg, float yLocal)
{
    const profile = cg.floorProfile;
    if (yLocal <= profile[0].y)
        return profile[0].z;
    if (yLocal >= profile[$ - 1].y)
        return profile[$ - 1].z;
    foreach (i; 1 .. profile.length)
        if (yLocal <= profile[i].y)
        {
            const float t = (yLocal - profile[i - 1].y)
                / (profile[i].y - profile[i - 1].y);
            return profile[i - 1].z + (profile[i].z - profile[i - 1].z) * t;
        }
    assert(false);
}

private bool beamPiercesCabin(const vec3 a, const vec3 b,
    const vec3 lo, const vec3 hi, const CockpitGeometry cg,
    const vec3 node0)
{
    const vec3 d = b - a;
    float tmin = 0.0f, tmax = 1.0f;
    if (!slab(tmin, tmax, a.x, d.x, lo.x, hi.x)) return false;
    if (!slab(tmin, tmax, a.y, d.y, lo.y, hi.y)) return false;
    if (!slab(tmin, tmax, a.z, d.z, -float.max, hi.z)) return false;
    if (!(tmin < tmax))
        return false;

    // Изломы профиля пола и границы окна — кандидаты на максимум
    // g(t) = z(t) − пол(y(t)); внутри каждого вдоль-линейного куска максимум
    // достигается на его концах.
    float[3 + 8] tPts;
    tPts[0] = tmin;
    size_t n = 1;
    if (abs(d.y) > 1e-12f)
        foreach (s; cg.floorProfile)
        {
            const float t = (node0.y + s.y - a.y) / d.y;
            if (t > tmin + 1e-9f && t < tmax - 1e-9f)
            {
                assert(n + 1 < tPts.length, "станций больше, чем ждём");
                tPts[n++] = t;
            }
        }
    tPts[n++] = tmax;
    sort(tPts[0 .. n]);

    foreach (i; 0 .. n)
    {
        const vec3 p = a + d * tPts[i];
        const float g = p.z - (node0.z + cabinFloor(cg, p.y - node0.y));
        if (g > 1e-6f)
            return true;
    }
    return false;
}

/// Слэб-тест одной оси: сужает [tmin, tmax] на пересечение луча с полосой.
private bool slab(ref float tmin, ref float tmax,
    float p, float d, float lo, float hi)
{
    if (abs(d) < 1e-12f)
        return p > lo && p < hi;
    float t0 = (lo - p) / d;
    float t1 = (hi - p) / d;
    if (t0 > t1)
    {
        const float t = t0; t0 = t1; t1 = t;
    }
    if (t0 > tmin) tmin = t0;
    if (t1 < tmax) tmax = t1;
    return tmin < tmax;
}

private bool wheelPiercesCabin(const vec3 p, float r,
    const vec3 lo, const vec3 hi, const CockpitGeometry cg,
    const vec3 node0)
{
    if (!(p.x >= lo.x - r && p.x <= hi.x + r
        && p.y >= lo.y - r && p.y <= hi.y + r))
        return false;
    const float floorZ = node0.z + cabinFloor(cg, p.y - node0.y);
    return p.z + r > floorZ && p.z - r < hi.z;
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

    // Хребет: станции по центру, повторяют контур днища (наклон от кормы
    // к носу), все ниже центра масс.
    assert(g.floorProfile.length == 5, "пять станций под днищем");
    foreach (s; g.floorProfile)
    {
        assert(abs(s.x) < 1e-4f, "станции по оси ширины виляют");
        assert(s.z <= 0.0f && s.z >= g.minP.z, "станции на контуре днища");
    }
    // Станции упорядочены от носа к корме, без повторов y; контур дна при
    // этом монотонно понижается к корме.
    float prevY = -float.max, prevZ = float.max;
    foreach (s; g.floorProfile)
    {
        assert(s.y > prevY, "станции идут от носа к корме");
        prevY = s.y;
        assert(s.z <= prevZ + 1e-4f, "контур днища без подъёмов к носу");
        prevZ = s.z;
    }

    // Боковые пары: в станциях, где рёбра днища параллельны ширине.
    assert(g.mountPairStations == [0, 2, 4], "боковые пары на параллельных рёбрах");
    assert(g.mountPairs.length == g.mountPairStations.length);
    foreach (i; 0 .. g.mountPairs.length)
    {
        const vec3 right = g.mountPairs[i][0], left = g.mountPairs[i][1];
        const size_t sIdx = g.mountPairStations[i];
        assert(right.x > 0.0f && right.x == -left.x, "пара зеркальна");
        assert(abs(right.y - g.floorProfile[sIdx].y) < 1e-4f, "пара в станции");
        const vec3 half = g.maxP;
        assert(right.x <= half.x && right.x > 0.0f, "пара в раскрыве корпуса");
    }

    const context = cockpitFrameContext();
    const vec3[12] expectedNodes = [
        vec3(0.0f, 0.0f, 0.0f),
        vec3(0.0f, -1.405f, -0.299f),
        vec3(0.0f, -0.82435484f, -0.32319355f),
        vec3(0.0f, -0.205f, -0.349f),
        vec3(0.0f, 0.22485075f, -0.39677612f),
        vec3(0.0f, 0.695f, -0.449f),
        vec3(0.30f, -1.405f, -0.299f),
        vec3(-0.30f, -1.405f, -0.299f),
        vec3(0.32f, -0.205f, -0.349f),
        vec3(-0.32f, -0.205f, -0.349f),
        vec3(0.35f, 0.695f, -0.449f),
        vec3(-0.35f, 0.695f, -0.449f),
    ];
    foreach (i, expected; expectedNodes)
        assert(distance(context.frame.nodes[i].pos, expected) < 1e-4f);

    const size_t[12] expectedTwins = [0, 1, 2, 3, 4, 5, 7, 6, 9, 8, 11, 10];
    foreach (i, expected; expectedTwins)
        assert(context.twinOf[i] == expected);

    const size_t[22] expectedBeamEnds = [
        0, 3, 1, 2, 2, 3, 3, 4, 4, 5,
        1, 6, 1, 7, 3, 8, 3, 9, 5, 10, 5, 11,
    ];
    foreach (i, beam; context.frame.beams)
    {
        assert(beam.a == expectedBeamEnds[i * 2]);
        assert(beam.b == expectedBeamEnds[i * 2 + 1]);
    }
    assert(context.growthNode == 11 && context.inertNode == 0);
    assert(context.frame.nodes.length == cockpitFrameNodeCount());
    assert(context.frame.beams.length == cockpitFrameBeamCount());

    // Константы дизайна на месте.
    assert(cockpitMass == 100.0f);
    assert(cockpitInertia.x == 16.7f && cockpitInertia.y == 8.5f
        && cockpitInertia.z == 16.7f);
}
