module genetics.fitness;

import std.math;
import std.algorithm : min, max, clamp;

import dlib.math.vector;

import frame.frame;

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

/// Оценочная фитнес-функция каркаса (без физики).
///
/// Возвращает 0 для физически невыполнимых каркасов и значение в (0,1]
/// для правдоподобных. Слои оценки (симметрия — это мягкое предпочтение
/// с нижней границей 0.5, чтобы асимметричный каркас не обнулялся):
///   - симметрия через плоскость X=0 (канализация из грамматики);
///   - жёсткость — петли в графе балок (цикломатическое число);
///   - устойчивость — низкий центр масс при широкой колее;
///   - колёсная база — продольный разброс колёс;
///   - плоскостность колёс по высоте;
///   - компактность — наказание за декоративные тупиковые балки;
///   - баланс ведущих колёс по сторонам.
float buggyFitness(const Frame f)
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

    // Центр масс горизонтально — внутри опорного многоугольника колёс.
    if (com.x < xmin - epsFlat || com.x > xmax + epsFlat)
        return 0.0f;
    if (com.y < ymin - epsFlat || com.y > ymax + epsFlat)
        return 0.0f;

    // ---- Слоты морфологии ----
    const float xspan = xmax - xmin;
    const float ybase = ymax - ymin;

    const float nodeSym = symmetryRatio(f);
    const float wheelSym = wheelSymmetry(f);
    const float phiSym = 0.5f + 0.5f * (0.5f * (nodeSym + wheelSym));

    const size_t cycles = cyclomaticNumber(f); // μ = E - V + c
    const float phiRigid = 0.5f + 0.5f * (1.0f - exp(-0.4f * cast(float) cycles));

    const float phiStab = 1.0f / (1.0f + clearance / max(xspan, 1e-3f));

    const float phiAxis = ramp(ybase, 0.15f, 0.6f)
        * sigmoid(ybase / max(xspan, 1e-3f) - 0.8f);

    const float phiFlat = 1.0f - clamp(zrange / maxWheelZRange, 0.0f, 1.0f);

    const size_t Vf = cast(size_t) V;
    const float deadRatio = cast(float) nonAnchorLeaves(f) / Vf;
    const float phiCompact = exp(-2.0f * deadRatio);

    const float phiDrive = 0.5f + 0.5f * motorBalance(f);

    return phiSym * phiRigid * phiStab * phiAxis * phiFlat * phiCompact * phiDrive;
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
    import genetics.encoder;

    // Стартовый закодированный багги — правдоподобный каркас.
    auto grammar = buggyGrammar();
    auto frame = develop(grammar, startGenome(grammar)).get;
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

    // Симметричная машина должна оцениваться выше асимметричной той же формы.
    const float symFitness = buggyFitness(symmetricBuggyFrame());
    const float asymFitness = buggyFitness(asymmetricBuggyFrame());
    assert(symFitness > 0.0f && asymFitness > 0.0f,
        "обе машины физически выполнимы");
    assert(symFitness > asymFitness,
        "зеркальность колёс и каркаса даёт прирост фитнеса");
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