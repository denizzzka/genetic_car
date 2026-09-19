module genetics.fitness;

import std.math;
import std.algorithm : min, max, clamp, sort;
import std.typecons : Tuple, tuple;

import dlib.math.vector;

import frame.frame;
import genetics.buggyast;
import physics_world;

/*
 * Статическая фитнес-функция — суррогат физики.
 *
 * Пока симуляция не встроена в цикл отбора, оцениваем каркас геометрически:
 * не "поедет ли", а "похож ли на машину и не развалится ли сразу". Форма —
 * гейт V и набор множителей φᵢ ∈ (0,1]:
 *
 *     fitness = V · Π φᵢ
 *
 * Умножение, а не сумма: ни один "бонус" не компенсирует недоделку.
 * Когда физика заработает, V·φ заменится на пройденное расстояние, а слоты
 * останутся как сглаживающий член.
 */

/// Порог "узел на плоскости симметрии" (|x| < epsFlat) и порог силы.
enum float epsFlat = 1e-4f;

/// Радиус колеса, совпадает с physics_world.physics.wheelRadius.
/// Нужен, чтобы перейти от центра колёс к плоскости земли (z = zmin - fitnessWheelRadius).
enum float fitnessWheelRadius = 0.3f;

/// Максимальный разброс высот колёс, при котором каркас ещё "стоит на полу".
enum float maxWheelZRange = 0.6f;

/// Допустимый клиренс (высота центра масс над нижней точкой колёс).
enum float minClearance = 0.02f;
enum float maxClearance = 2.0f;

/// Разумный потолок сложности каркаса.
enum size_t maxBeamCount = 64;

/// Целевые стороны footprint колёс (минимальный прямоугольник в плоскости
/// XY, без фиксации ориентации) и допуск.
enum float targetFootprintLong = 3.0f;
enum float targetFootprintShort = 2.0f;
enum float footprintTolerance = 1.0f;

/// Целевая высота габаритного параллелепипеда каркаса: заполнение объёма
/// вверх (размах узлов по Z) и допуск.
enum float targetFrameHeight = 2.5f;
enum float frameHeightTolerance = 0.5f;

/// Допуски положения центра масс внутри габарита: по XY — к центру
/// колёсного footprint, по Z — к верхней границе клиренса (maxClearance).
enum float comXYTolerance = 0.5f;
enum float comZTolerance = 0.5f;

/// Оценочная фитнес-функция каркаса (без физики).
///
/// Возвращает 0 для физически невыполнимых каркасов и значение в (0,1]
/// для правдоподобных. Слои оценки (симметрия — это мягкое предпочтение
/// с нижней границей 0.5, чтобы асимметричный каркас не обнулялся):
///   - симметрия через плоскость X=0 (канализация из грамматики);
///   - жёсткость — петли в графе балок (цикломатическое число);
///   - положение центра масс — к центру габарита на земле, как можно выше;
///   - колёсная база — продольный разброс колёс;
///   - плоскостность колёс по высоте;
///   - компактность — наказание за декоративные тупиковые балки;
///   - баланс ведущих колёс по сторонам;
///   - габариты — footprint колёс;
///   - заполнение высоты — размах узлов каркаса по Z;
///   - габаритный параллелепипед — штраф за узлы за пределами
///     (колёсный AABB по XY + высота от земли до целевой).
float buggyFitness(const Frame f)
{
    return buggyFitness(f, Ast.init);
}

float buggyFitness(const Frame f, const Ast ast)
{
    // ---- Гейт: физическая выполнимость ----
    if (f.nodes.length == 0 || f.beams.length == 0)
        return 0.0f;
    if (f.beams.length > maxBeamCount)
        return 0.0f;
    if (!isConnected(f))
        return 0.0f;
    if (f.anchors.length < 2)
        return 0.0f;

    const size_t V = f.nodes.length;

    // Аппроксимация центра масс: масса балки ∝ r²·len, центр — середина.
    vec3 com = vec3(0.0f);
    float totalMass = 0.0f;
    foreach (b; f.beams)
    {
        const vec3 a = f.nodes[b.a].pos;
        const vec3 d = f.nodes[b.b].pos - a;
        const float len = d.length;
        if (len < 1e-5f)
            continue;
        const float m = b.radius * b.radius * len;
        com += (a + 0.5f * d) * m;
        totalMass += m;
    }
    if (totalMass <= 0.0f)
        return 0.0f;
    com /= totalMass;

    // Разброс колёс и число ведущих.
    float xmin = float.max, xmax = -float.max;
    float ymin = float.max, ymax = -float.max;
    float zmin = float.max, zmax = -float.max;
    size_t nMotor = 0;
    foreach (a; f.anchors)
    {
        const vec3 p = f.nodes[a.node].pos;
        xmin = min(xmin, p.x); xmax = max(xmax, p.x);
        ymin = min(ymin, p.y); ymax = max(ymax, p.y);
        zmin = min(zmin, p.z); zmax = max(zmax, p.z);
        if (a.kind == AnchorKind.motorWheel)
            nMotor += 1;
    }
    if (nMotor == 0)
        return 0.0f;

    const float zrange = zmax - zmin;
    if (zrange > maxWheelZRange)
        return 0.0f;

    // Клиренс: высота центра масс над плоскостью земли. Земля — под
    // нижней точкой колёс: z = zmin - fitnessWheelRadius.
    const float groundZ = zmin - fitnessWheelRadius;
    const float clearance = com.z - groundZ;
    if (clearance < minClearance || clearance > maxClearance)
        return 0.0f;

    // Балки не должны торчать ниже колёс: ни один узел каркаса не опускается
    // под плоскость земли. Допуск epsFlat прощает касание, но не проникновение.
    foreach (n; f.nodes)
        if (n.pos.z < groundZ - epsFlat)
            return 0.0f;

    // Центр масс горизонтально — внутри опорного многоугольника колёс.
    if (com.x < xmin - epsFlat || com.x > xmax + epsFlat)
        return 0.0f;
    if (com.y < ymin - epsFlat || com.y > ymax + epsFlat)
        return 0.0f;

    // ---- Слоты морфологии ----
    const float xspan = xmax - xmin;
    const float ybase = ymax - ymin;

    const dims = footprintExtents(f);
    const float phiFootprint = exp(-((dims[0] - targetFootprintLong) / footprintTolerance) ^^ 2)
        * exp(-((dims[1] - targetFootprintShort) / footprintTolerance) ^^ 2);

    // Заполнение высоты: размах узлов каркаса по Z (не только колёс).
    const float height = frameZRange(f);
    const float phiHeight = exp(-((height - targetFrameHeight) / frameHeightTolerance) ^^ 2);

    const float nodeSym = symmetryRatio(f);
    const float wheelSym = wheelSymmetry(f);
    const float pairSym = beamMassSymmetry(f);
    const float forkSym = forkRadiusSymmetry(ast);
    const float base = 0.5f * nodeSym + 0.3f * wheelSym + 0.2f * pairSym;
    const float phiSym = (0.5f + 0.5f * base) * forkSym;

    const size_t cycles = cyclomaticNumber(f); // μ = E - V + c
    const float phiRigid = 0.5f + 0.5f * (1.0f - exp(-0.4f * cast(float) cycles));

    const float phiCoM = comCentering(f, com);

    const float phiAxis = ramp(ybase, 0.15f, 0.6f)
        * sigmoid(ybase / max(xspan, 1e-3f) - 0.8f);

    const float phiFlat = 1.0f - clamp(zrange / maxWheelZRange, 0.0f, 1.0f);

    const size_t Vf = cast(size_t) V;
    const float deadRatio = cast(float) nonAnchorLeaves(f) / Vf;
    const float phiCompact = exp(-2.0f * deadRatio);

    const float phiDrive = 0.5f + 0.5f * motorBalance(f);

    // Балки за габаритом: доля узлов за пределами параллелепипеда
    // (колёсный AABB по XY + от земли до целевой высоты по Z)
    // гасит фитнес экспоненциально — параллелепипед не должен
    // обрастать лишними элементами за пределами габарита.
    float nodesOutsideGauge = 0;
    foreach (n; f.nodes)
    {
        const bool inX = n.pos.x >= xmin - epsFlat && n.pos.x <= xmax + epsFlat;
        const bool inY = n.pos.y >= ymin - epsFlat && n.pos.y <= ymax + epsFlat;
        const bool inZ = n.pos.z >= groundZ - epsFlat && n.pos.z <= groundZ + targetFrameHeight + epsFlat;
        if (!(inX && inY && inZ))
            nodesOutsideGauge += 1.0f;
    }
    const float gaugeViolation = nodesOutsideGauge / cast(float) f.nodes.length;
    const float phiGauge = exp(-3.0f * gaugeViolation);

    return phiSym * phiRigid * phiCoM * phiAxis * phiFlat * phiCompact * phiDrive
        * phiFootprint * phiHeight * phiGauge;
}

/// Параметры заезда (физический слой оценки).
enum double physicsDt = 1.0 / 60.0;       ///< шаг симуляции (стабильный dt)
enum double physicsSimSeconds = 3.0;      ///< длительность заезда особи, сек
enum float physicsNominalSpeed = 2.0f;    ///< м/с фитнеса — дистанция-норма
enum float physicsSlopeDeg = 25.0f;       ///< уклон «горки» (наклон вектора g)
enum double physicsSettleSeconds = 1.0;   ///< успокоение осадки перед стартом

/// Итог физического заезда одной машины.
///
/// Помимо мультипликатора `score` в фитнес отдаёт телеметрию для вывода:
/// фактический достигнутый спуск (`descent`), сколько было колёс/балок и что
/// оборвало заезд (`why`), если `survived` ложно.
struct PhysicsResult
{
    float score = 0.0f;    // вклад в фитнес: (0..1]; 0 — не доехала
    bool survived = false; // заезд дошёл до конца, не развалившись
    double descent = 0.0;  // достигнутый спуск по курсу (-Y), м
    size_t wheels = 0;     // выставленное число колёс
    size_t beams = 0;      // выставленное число балок
    string why = "";       // причина обрыва (пусто — успех)
}

/// Физический слой оценки: пассивный спуск с горки `physicsSlopeDeg`.
///
/// Силу тяжести наклоняют (`setSlopeDeg`), земля остаётся плоской; колёса на
/// осях катятся сами, газовая тяга не участвует. Счёт — продвижение центра
/// колёс BНИЗ по курсу (-Y) за `seconds` секунд, нормированное
/// на `physicsNominalSpeed·seconds` → (0,1]. Живучесть: любое колесо
/// провалилось под землю, зависло (переворот, съезд) или каркас разлетелся —
/// заезд обрывается, счёт 0. Симуляция строится на `BuggyPhysics` отдельно,
/// фитнес только читает её наружу.
PhysicsResult physicsRun(const Buggy buggy, double seconds)
{
    PhysicsResult r;
    r.wheels = buggy.frame.anchors.length;

    if (buggy.frame.anchors.length < 2)
    {
        r.why = "объект с одним колесом";
        return r;
    }

    auto physics = new BuggyPhysics(buggy);
    scope (exit) physics.dispose();

    physics.setSlopeDeg(physicsSlopeDeg);
    physics.settle(physicsDt,
        cast(int)(physicsSettleSeconds / physicsDt));

    const settleFailure = runFailure(physics);
    if (settleFailure.length)
    {
        r.why = settleFailure;
        return r;
    }

    const size_t steps = cast(size_t)(seconds / physicsDt);

    auto wheels = physics.wheelStates();
    r.beams = physics.beamStates().length;

    double startY = 0.0;
    foreach (s; wheels)
        startY += s.position.y;
    startY /= wheels.length;

    // Продвижение по спуску — максимум дистанции, преодолённой вниз (-Y).
    double farthest = 0.0;
    foreach (_; 0 .. steps)
    {
        physics.step(physicsDt, 0.0f);

        const stepFailure = runFailure(physics);
        if (stepFailure.length)
        {
            r.why = stepFailure;
            break;
        }

        wheels = physics.wheelStates();
        double curY = 0.0;
        foreach (s; wheels)
            curY += s.position.y;
        curY /= wheels.length;
        const double downhill = startY - curY;     // +Y — вверх по склону
        farthest = max(farthest, downhill);
    }

    r.descent = farthest;
    if (r.why.length != 0)
        return r;

    const double target = physicsNominalSpeed * seconds;
    r.survived = true;
    r.score = clamp(cast(float)(farthest / target), 0.0f, 1.0f);
    return r;
}

/// Мультипликатор фитнеса из физического заезда — краткая форма `physicsRun`.
float physicsFitness(const Buggy buggy, double seconds)
{
    return physicsRun(buggy, seconds).score;
}

/// Доля узлов, у которых есть зеркальный партнёр через плоскость X=0.
/// Узел на плоскости считается собственным зеркалом.
float symmetryRatio(const Frame f)
{
    const size_t V = f.nodes.length;
    const float diag = aabbDiagonal(f);
    const float eps = max(0.02f * diag, 1e-3f);

    bool[] used = new bool[V];
    size_t matched = 0;

    foreach (i; 0 .. V)
    {
        if (used[i])
            continue;
        const vec3 p = f.nodes[i].pos;
        const vec3 m = vec3(-p.x, p.y, p.z);

        if (distance(p, m) < eps) // узел на плоскости
        {
            used[i] = true;
            matched += 1;
            continue;
        }

        size_t best = V;
        float bestD = eps;
        foreach (j; 0 .. V)
            if (!used[j] && j != i)
            {
                const float d = distance(f.nodes[j].pos, m);
                if (d < bestD)
                {
                    bestD = d;
                    best = j;
                }
            }
        if (best < V)
        {
            used[i] = used[best] = true;
            matched += 2;
        }
    }

    return cast(float) matched / V;
}

/// Доля колёс, у которых есть колесо на зеркальной позиции.
float wheelSymmetry(const Frame f)
{
    const float diag = aabbDiagonal(f);
    const float eps = max(0.02f * diag, 1e-3f);

    size_t matched = 0;
    foreach (i, a; f.anchors)
    {
        const vec3 p = f.nodes[a.node].pos;
        const vec3 m = vec3(-p.x, p.y, p.z);
        if (distance(p, m) < eps)
        {
            matched += 1;
            continue;
        }
        foreach (j, b; f.anchors)
            if (j != i && distance(f.nodes[b.node].pos, m) < eps)
            {
                matched += 1;
                break;
            }
    }

    return cast(float) matched / f.anchors.length;
}

/// Доля массы балок, зеркально парных с балкой того же радиуса (по массе
/// `r²·len`), включая самосимметричные: балку на плоскости и балку,
/// пересекающую плоскость своими зеркальными концами. Штрафует и позиционную,
/// и радиальную (след Nodal/Lefty) асимметрию.
float beamMassSymmetry(const Frame f)
{
    if (f.beams.length == 0)
        return 1.0f;

    const float diag = aabbDiagonal(f);
    const float eps = max(0.02f * diag, 1e-3f);

    float mass(size_t i) {
        const vec3 a = f.nodes[f.beams[i].a].pos;
        const vec3 c = f.nodes[f.beams[i].b].pos;
        return f.beams[i].radius * f.beams[i].radius * (c - a).length;
    }

    bool[] used = new bool[f.beams.length];
    float total = 0.0f, matched = 0.0f;

    foreach (i; 0 .. f.beams.length)
    {
        const float m = mass(i);
        total += m;
        if (used[i])
        {
            matched += m;
            continue;
        }

        const vec3 a = f.nodes[f.beams[i].a].pos;
        const vec3 c = f.nodes[f.beams[i].b].pos;
        const vec3 ma = vec3(-a.x, a.y, a.z);
        const vec3 mc = vec3(-c.x, c.y, c.z);

        if ((distance(a, ma) < eps && distance(c, mc) < eps)
            || (distance(a, mc) < eps && distance(c, ma) < eps))
        {
            used[i] = true;
            matched += m;
            continue;
        }

        size_t best = f.beams.length;
        float bestD = eps;
        foreach (j; 0 .. f.beams.length)
        {
            if (j == i || used[j])
                continue;
            if (abs(f.beams[j].radius - f.beams[i].radius) > 0.02f * f.beams[i].radius + 1e-4f)
                continue;
            const vec3 da = f.nodes[f.beams[j].a].pos;
            const vec3 dc = f.nodes[f.beams[j].b].pos;
            const float d1 = distance(da, ma) + distance(dc, mc);
            const float d2 = distance(da, mc) + distance(dc, ma);
            if (min(d1, d2) < bestD)
            {
                bestD = min(d1, d2);
                best = j;
            }
        }
        if (best < f.beams.length)
        {
            used[i] = used[best] = true;
            matched += m;
        }
    }

    return total > 0.0f ? matched / total : 1.0f;
}

/// Радиальная симметрия fork-пар из AST: 1 при нулевом отклике
/// активатор-ингибитор (`|forkAsymmetry(nodal, lefty)|`) на всех балках
/// раздвоенных сегментов. На пустом AST (синтетические каркасы без генома) —
/// нейтрально 1.
float forkRadiusSymmetry(const Ast ast)
{
    float sum = 0.0f;
    size_t n = 0;
    foreach (s; ast.segments)
        if (s.fork)
            foreach (b; s.beams)
            {
                sum += abs(forkAsymmetry(b.nodal, b.lefty));
                n += 1;
            }
    if (n == 0)
        return 1.0f;

    enum penaltyFactor = 12.0f;
    return exp(-penaltyFactor * sum / n);
}

/// Цикломатическое число графа балок μ = E - V + c (число независимых петель).
size_t cyclomaticNumber(const Frame f)
{
    const size_t V = f.nodes.length;
    size_t[] parent = new size_t[V];
    foreach (i; 0 .. V)
        parent[i] = i;

    size_t root(size_t x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
    void join(size_t a, size_t b) { const ra = root(a); const rb = root(b); if (ra != rb) parent[ra] = rb; }

    foreach (b; f.beams)
        if (b.a < V && b.b < V)
            join(b.a, b.b);

    size_t comps = 0;
    foreach (i; 0 .. V)
        if (root(i) == i)
            comps += 1;

    // μ = E - V + c ⩾ 0: в каждой компоненте E ≥ V_c - 1 (петли дают прирост).
    const ptrdiff_t mu = cast(ptrdiff_t) f.beams.length + cast(ptrdiff_t) comps - cast(ptrdiff_t) V;
    return mu > 0 ? cast(size_t) mu : 0;
}

/// Вертикальный размах всех узлов каркаса (габаритная высота параллелепипеда).
float frameZRange(const Frame f)
{
    if (f.nodes.length == 0)
        return 0.0f;

    float lo = f.nodes[0].pos.z;
    float hi = f.nodes[0].pos.z;
    foreach (n; f.nodes)
    {
        lo = min(lo, n.pos.z);
        hi = max(hi, n.pos.z);
    }
    return hi - lo;
}

/// Положение центра масс в габаритном параллелепипеде: по XY — к центру
/// колёсного footprint, по Z — как можно выше (к верхней границе гейта
/// клиренса maxClearance). Множитель ∈ (0,1].
private float comCentering(const Frame f, const vec3 com)
{
    float xmin = float.max, xmax = -float.max;
    float ymin = float.max, ymax = -float.max;
    float zmin = float.max;
    foreach (a; f.anchors)
    {
        const vec3 p = f.nodes[a.node].pos;
        xmin = min(xmin, p.x); xmax = max(xmax, p.x);
        ymin = min(ymin, p.y); ymax = max(ymax, p.y);
        zmin = min(zmin, p.z);
    }
    const float cx = 0.5f * (xmin + xmax);
    const float cy = 0.5f * (ymin + ymax);
    const float targetZ = zmin - fitnessWheelRadius + maxClearance;

    return exp(-(((com.x - cx) / comXYTolerance) ^^ 2
        + ((com.y - cy) / comXYTolerance) ^^ 2
        + ((targetZ - com.z) / comZTolerance) ^^ 2));
}

/// Диагональ ограничивающего бокса всех узлов.
float aabbDiagonal(const Frame f)
{
    if (f.nodes.length == 0)
        return 0.0f;

    vec3 lo = f.nodes[0].pos;
    vec3 hi = f.nodes[0].pos;
    foreach (n; f.nodes)
    {
        lo.x = min(lo.x, n.pos.x); lo.y = min(lo.y, n.pos.y); lo.z = min(lo.z, n.pos.z);
        hi.x = max(hi.x, n.pos.x); hi.y = max(hi.y, n.pos.y); hi.z = max(hi.z, n.pos.z);
    }
    return (hi - lo).length;
}

/// Длинная и короткая стороны бокса выпуклой оболочки колёс в плоскости XY,
/// у ориентации, где он ближе всего к целевому прямоугольнику (2×3).
/// Ориентация каркаса не важна: повёрнутая «диагональ» не раздувает стороны.
Tuple!(float, float) footprintExtents(const Frame f)
{
    vec3[] pts;
    pts.reserve(f.anchors.length);
    foreach (a; f.anchors)
        pts ~= f.nodes[a.node].pos;

    auto crossZ = (vec3 o, vec3 a, vec3 b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };

    sort!"a.x != b.x ? a.x < b.x : a.y < b.y"(pts);

    vec3[] hull;
    vec3[] lower;
    foreach (p; pts)
    {
        while (lower.length >= 2 && crossZ(lower[$ - 2], lower[$ - 1], p) <= 0.0f)
            lower.length -= 1;
        lower ~= p;
    }
    vec3[] upper;
    foreach_reverse (p; pts)
    {
        while (upper.length >= 2 && crossZ(upper[$ - 2], upper[$ - 1], p) <= 0.0f)
            upper.length -= 1;
        upper ~= p;
    }
    if (lower.length >= 2 && upper.length >= 2)
        hull = lower[0 .. $ - 1] ~ upper[0 .. $ - 1];

    if (hull.length < 3)
    {
        float longest = 0.0f;
        foreach (p; pts)
            foreach (q; pts)
                longest = max(longest, distance(p, q));
        return tuple(longest, 0.0f);
    }

    // Ориентация, чей box ближе всего к целевому прямоугольнику.
    float bestDist = float.max;
    float longSide, shortSide;
    foreach (i; 0 .. hull.length)
    {
        const a = hull[i];
        const b = hull[(i + 1) % hull.length];
        vec3 e = b - a;
        const el = e.length;
        if (el < 1e-6f)
            continue;
        e /= el;

        float minU = float.max, maxU = -float.max;
        float minV = float.max, maxV = -float.max;
        foreach (p; hull)
        {
            const u = e.x * p.x + e.y * p.y;
            const v = -e.y * p.x + e.x * p.y;
            minU = min(minU, u); maxU = max(maxU, u);
            minV = min(minV, v); maxV = max(maxV, v);
        }
        const w = maxU - minU;
        const h = maxV - minV;
        const lo = min(w, h);
        const hi = max(w, h);
        const dist = (hi - targetFootprintLong) ^^ 2 + (lo - targetFootprintShort) ^^ 2;
        if (dist < bestDist)
        {
            bestDist = dist;
            longSide = hi;
            shortSide = lo;
        }
    }
    return tuple(longSide, shortSide);
}

/// Число "декоративных" тупиков: узлы степени 1 без колеса.
size_t nonAnchorLeaves(const Frame f)
{
    const size_t V = f.nodes.length;
    size_t[] deg = new size_t[V];
    bool[] hasAnchor = new bool[V];

    foreach (b; f.beams)
    {
        deg[b.a] += 1;
        deg[b.b] += 1;
    }
    foreach (a; f.anchors)
        hasAnchor[a.node] = true;

    size_t leaves = 0;
    foreach (i; 0 .. V)
        if (deg[i] == 1 && !hasAnchor[i])
            leaves += 1;
    return leaves;
}

/// Баланс ведущих колёс по сторонам: 1 — парами, 0 — только с одной стороны.
/// Колёса на плоскости симметрии не считаются.
float motorBalance(const Frame f)
{
    size_t left = 0, right = 0;
    foreach (a; f.anchors)
        if (a.kind == AnchorKind.motorWheel)
        {
            const float x = f.nodes[a.node].pos.x;
            if (x > epsFlat)
                right += 1;
            else if (x < -epsFlat)
                left += 1;
        }

    const size_t total = left + right;
    return 1.0f - cast(float) (left > right ? left - right : right - left) / max(1.0f, cast(float) total);
}

private float ramp(float v, float lo, float hi)
{
    return clamp((v - lo) / max(hi - lo, 1e-6f), 0.0f, 1.0f);
}

private float sigmoid(float x)
{
    return 1.0f / (1.0f + exp(-x));
}

unittest
{
    import genetics.sge;
    import genetics.buggygrammar;
    import genetics.initial_data;

    // Стартовый закодированный багги — правдоподобный каркас.
    auto grammar = buggyGrammar();
    auto frame = develop(grammar, startGenome(grammar)).get.frame;
    assert(buggyFitness(frame) > 0.0f,
        "стартовый багги должен получать положительный фитнес");

    // Пустой каркас — не машина.
    Frame emptyFrame;
    assert(buggyFitness(emptyFrame) == 0.0f);

    // Каркас без колёс — 0.
    Frame noWheels;
    noWheels.nodes ~= Node(vec3(0.0f, 0.0f, 0.0f));
    noWheels.nodes ~= Node(vec3(0.0f, 1.0f, 0.0f));
    noWheels.beams ~= Beam(0, 1, 0.05f);
    assert(buggyFitness(noWheels) == 0.0f);

    // Колеса есть, но нет ведущего — 0.
    Frame noMotor = noWheels;
    noMotor.anchors ~= Anchor(0, AnchorKind.wheel);
    noMotor.anchors ~= Anchor(1, AnchorKind.wheel);
    assert(buggyFitness(noMotor) == 0.0f);

    // Разорванный каркас (две несвязанные половины) — 0.
    Frame split = noMotor;
    split.nodes ~= Node(vec3(5.0f, 5.0f, 0.3f));
    split.nodes ~= Node(vec3(5.0f, 6.0f, 0.3f));
    split.beams ~= Beam(2, 3, 0.04f);
    assert(!isConnected(split));
    assert(buggyFitness(split) == 0.0f);

    // Балки, торчащие ниже колёс: узел опускается под плоскость земли
    // (под нижнюю точку колёс) — физическая отбраковка.
    Frame underGround = symmetricBuggyFrame();
    underGround.nodes ~= Node(vec3(0.0f, 0.0f, -1.0f));
    underGround.beams ~= Beam(0, underGround.nodes.length - 1, 0.04f);
    assert(buggyFitness(underGround) == 0.0f,
        "балка ниже уровня земли должна отбраковываться");

    // Тот же каркас с узлом, лишь касающимся земли (допуск), — не отбраковка.
    Frame boundary = symmetricBuggyFrame();
    // Нижняя точка колёс: zmin колёс = 0.25 -> земля 0.25 - 0.3 = -0.05.
    boundary.nodes ~= Node(vec3(0.0f, 0.0f, -0.05f));
    boundary.beams ~= Beam(0, boundary.nodes.length - 1, 0.04f);
    assert(buggyFitness(boundary) > 0.0f,
        "касание плоскости земли в пределах допуска не отбраковывается");

    // Симметричная машина должна оцениваться выше асимметричной той же формы.
    const float symFitness = buggyFitness(symmetricBuggyFrame());
    const float asymFitness = buggyFitness(asymmetricBuggyFrame());
    assert(symFitness > 0.0f && asymFitness > 0.0f,
        "обе машины физически выполнимы");
    assert(symFitness > asymFitness,
        "зеркальность колёс и каркаса даёт прирост фитнеса");

    // Заполнение высоты: каркас с вертикальным размахом ближе к габаритной
    // высоте оценивается выше плоского той же колёсной базы.
    const float flat = buggyFitness(symmetricBuggyFrame());
    const float tall = buggyFitness(tallBuggyFrame());
    assert(tall > 0.0f, "высокий каркас физически выполним");
    assert(tall > flat, "поощрение заполнения высоты габаритного параллелепипеда");

    // Балки за габаритом: идентичные каркасы, отличающиеся только одним узлом
    // (внутри колёсного AABB против выступающего наружу), — выступающий
    // получает меньший фитнес. deadRatio у обоих одинаков (1/7), отличие — phiGauge.
    Frame inside = symmetricBuggyFrame();
    inside.nodes ~= Node(vec3(0.3f, 0.0f, 0.4f));
    inside.beams ~= Beam(0, inside.nodes.length - 1, 0.04f);
    Frame outside = symmetricBuggyFrame();
    outside.nodes ~= Node(vec3(1.5f, 0.0f, 0.4f));
    outside.beams ~= Beam(0, outside.nodes.length - 1, 0.04f);
    assert(outside.nodes[$ - 1].pos.x > 0.7f + epsFlat,
        "узел теста обязан выступать за X AABB колёс (0.7)");
    assert(buggyFitness(inside) > 0.0f && buggyFitness(outside) > 0.0f);
    assert(buggyFitness(inside) > buggyFitness(outside),
        "балка за пределами габаритного параллелепипеда должна понижать фитнес");
}

unittest
{
    // Парность массы: симметричный багги полностью парен, асимметричный — нет.
    assert(beamMassSymmetry(symmetricBuggyFrame()) > 0.99f,
        "симметричный багги — полная зеркальная парность массы");
    assert(beamMassSymmetry(asymmetricBuggyFrame()) < 0.99f,
        "асимметричный багги не имеет пары части массы");
}

unittest
{
    // Nodal/Lefty из AST: тот же каркас, но AST сообщает о радиальном разбросе
    // fork-пары — ненулевой |forkAsymmetry| снижает фитнес. Пустой AST нейтрален.
    const base = buggyFitness(symmetricBuggyFrame());
    assert(base > 0.0f);

    Ast a0;
    a0.segments ~= SegmentAst(true, 0.0f, []);
    Ast aD;
    // Ненулевой активатор без ингибитора при ровной паре рождает сдвиг twin.
    aD.segments ~= SegmentAst(true, 0.0f, []);
    aD.segments[0].beams ~= BeamAst(StartRef(StartRefKind.last, 0),
        EndRef(EndRefKind.newNode, vec3(0.0f, 0.0f, 0.0f), 0),
        0.04f, 0.1f, 0.0f, BeamKind.normal, 0.0f);

    const f0 = buggyFitness(symmetricBuggyFrame(), a0);
    const fD = buggyFitness(symmetricBuggyFrame(), aD);
    assert(abs(f0 - base) < 1e-6f, "нулевой Nodal/Lefty не меняет фитнес");
    assert(fD < f0, "ненулевой |forkAsymmetry| штрафует асимметрию fork-пары");
}

unittest
{
    // comCentering: тот же каркас — выше и ближе к центру footprint лучше.
    const f = symmetricBuggyFrame();
    const high  = vec3(0.0f, 0.0f, 2.0f);
    const low   = vec3(0.0f, 0.0f, 0.5f);
    const side  = vec3(0.6f, 0.0f, 0.5f);
    assert(comCentering(f, high) > comCentering(f, low),
        "ЦТ выше — ближе к целевому положению");
    assert(comCentering(f, high) > comCentering(f, side),
        "ЦТ ближе к центру footprint — лучше");
}

unittest
{
    // Физический слой: заезд простейшего багги конечен, счёт нормирован
    // в (0,1] и не зависит от статики. Пустой каркас — ровно 0.
    const p = physicsFitness(new Buggy(symmetricBuggyFrame(), vec3(0.0f)), 1.0);
    assert(isFinite(p) && p >= 0.0f && p <= 1.0f,
        "счёт заезда нормирован и не разлетается");

    Frame empty;
    assert(physicsFitness(new Buggy(empty, vec3(0.0f)), 1.0) == 0.0f,
        "каркас без колёс не выезжает из нуля");
}

/// Симметричная машина, дополненная вертикальной надстройкой: тот же footprint,
/// но размах узлов по Z ближе к целевой высоте 2.5 м.
private Frame tallBuggyFrame()
{
    Frame f = symmetricBuggyFrame();
    f.nodes ~= Node(vec3(0.0f, 0.0f, 2.5f));
    f.beams ~= Beam(0, f.nodes.length - 1, 0.045f); // от центра (узел 0) вверх
    return f;
}

unittest
{
    // footprintExtents — box оболочки, ближайший к целевому 2×3; поворот
    // каркаса на 45° не должен увеличивать стороны.
    auto wheels = [tuple(0.7f, 0.7f), tuple(-0.7f, 0.7f), tuple(-0.7f, -0.7f), tuple(0.7f, -0.7f)];

    Frame sq;
    foreach (p; wheels)
    {
        sq.nodes ~= Node(vec3(p[0], p[1], 0.3f));
        sq.anchors ~= Anchor(sq.nodes.length - 1, AnchorKind.wheel);
    }
    const d1 = footprintExtents(sq);
    assert(abs(d1[0] - 1.4f) < 1e-3f && abs(d1[1] - 1.4f) < 1e-3f,
        "осевая квадратная оболочка 1.4×1.4");

    Frame rot;
    const float c = cos(PI / 4.0f), s = sin(PI / 4.0f);
    foreach (p; wheels)
    {
        const float x = p[0] * c - p[1] * s;
        const float y = p[0] * s + p[1] * c;
        rot.nodes ~= Node(vec3(x, y, 0.3f));
        rot.anchors ~= Anchor(rot.nodes.length - 1, AnchorKind.wheel);
    }
    const d2 = footprintExtents(rot);
    assert(abs(d2[0] - 1.4f) < 1e-2f && abs(d2[1] - 1.4f) < 1e-2f && d2[0] < 1.6f,
        "поворот на 45° не увеличивает footprint: диагональ не проходит");
}

private Frame symmetricBuggyFrame()
{
    Frame f;
    size_t node(vec3 p)
    {
        f.nodes ~= Node(p);
        return f.nodes.length - 1;
    }

    const c = node(vec3(0.0f, 0.0f, 0.4f));
    const fl = node(vec3(0.7f, 0.6f, 0.3f));
    const fr = node(vec3(-0.7f, 0.6f, 0.3f));
    const rl = node(vec3(0.6f, -0.6f, 0.25f));
    const rr = node(vec3(-0.6f, -0.6f, 0.25f));

    f.beams ~= Beam(c, fl, 0.045f);
    f.beams ~= Beam(c, fr, 0.045f);
    f.beams ~= Beam(c, rl, 0.05f);
    f.beams ~= Beam(c, rr, 0.05f);
    f.beams ~= Beam(fl, fr, 0.045f); // передняя ось: петля
    f.beams ~= Beam(rl, rr, 0.05f);  // задняя ось: петля

    f.anchors ~= Anchor(fl, AnchorKind.wheel);
    f.anchors ~= Anchor(fr, AnchorKind.wheel);
    f.anchors ~= Anchor(rl, AnchorKind.motorWheel);
    f.anchors ~= Anchor(rr, AnchorKind.motorWheel);
    return f;
}

private Frame asymmetricBuggyFrame()
{
    Frame f;
    size_t node(vec3 p)
    {
        f.nodes ~= Node(p);
        return f.nodes.length - 1;
    }

    const c = node(vec3(0.0f, 0.0f, 0.4f));
    const fl = node(vec3(0.7f, 0.6f, 0.3f));
    const fr = node(vec3(-0.7f, 0.6f, 0.3f));
    const rl = node(vec3(0.6f, -0.6f, 0.25f));

    f.beams ~= Beam(c, fl, 0.045f);
    f.beams ~= Beam(c, fr, 0.045f);
    f.beams ~= Beam(c, rl, 0.05f);
    f.beams ~= Beam(fl, fr, 0.045f);

    f.anchors ~= Anchor(fl, AnchorKind.wheel);
    f.anchors ~= Anchor(fr, AnchorKind.wheel);
    f.anchors ~= Anchor(rl, AnchorKind.motorWheel);
    return f;
}