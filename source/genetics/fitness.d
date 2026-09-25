module genetics.fitness;

import std.algorithm : min, max, clamp, sort;
import std.math;

import dlib.math.vector;

import frame.frame;
import frame.cockpit : cockpitGeometry, CockpitGeometry;
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

/// Малейший гарантированный зазор между колёсными цилиндрами в точке старта:
/// два почти совпадающих цилиндра (колесо-двойник на одном узле) валят GJK
/// в Newton 3.14, поэтому разнесение нижних ободов меньше этого порога
/// отбраковывается геометрически.
enum float wheelWheelMinGap = 1e-3f;

/// Кабина — неприкосновенна: никто (земля, балки, колёса) не должен касаться
/// её корпуса. Запретная зона — строгая внутренность параллелепипеда `dims`
/// вокруг узла 0 (низ кабины совмещён с узлом 0). Пол кабины — граница зоны:
/// балка в самой плоскости пола зону не задевает, эволюция сама расставит
/// балки ниже.

/// Разумный потолок сложности каркаса.
enum size_t maxBeamCount = 64;

/// Габаритный ящик каркаса — единый односторонний потолок размера:
/// 3 по курсу (размах) × 2.5 поперёк × 2.5 над землёй, центр встаёт на
/// середину колёсного footprint. Каркас меньше ящика ничем не наказывается;
/// узлы и колёса за его гранью штрафуются долей (phiBox).
enum float boxCourseHalf = 1.5f;
enum float boxWidthHalf = 1.25f;
enum float boxHeight = 2.5f;

/// Нижний порог морфологического множителя phiBox: суррогат
/// лишь сглаживает отбор, а не зануляет каркас. Основатель, вылезший за
/// ящик, получает порог вместо ~e^-25 и остаётся видимым отбору и физике.
enum float morphologyFloor = 0.01f;

/// Оценочная фитнес-функция каркаса (без физики).
///
/// Возвращает 0 для физически невыполнимых каркасов и значение в (0,1]
/// для правдоподобных. Слои оценки (симметрия — это мягкое предпочтение
/// с нижней границей 0.5, чтобы асимметричный каркас не обнулялся):
///   - симметрия через плоскость X=0 (канализация из грамматики);
///   - жёсткость — петли в графе балок (цикломатическое число);
///   - баланс ведущих колёс по сторонам;
///   - габаритный ящик — один параллелепипед 3×2.5×2.5 вокруг центра
///     колёс: узлы за его гранью штрафуются долей.
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

    // Два якоря на одном узле (или близко друг к другу) дают почти совпадающие
    // коллайдеры: на вырожденном Minkowski-hull двух идентичных цилиндров
    // Newton 3.14 рвёт свою книгу граней (dgContactSolver) и падает с SIGSEGV
    // ещё до contact-колбэка, поэтому каркас, чьи нижние ободы уже в точке
    // старта задевают друг друга (зазор по своим радиусам меньше порога),
    // отбраковываем геометрически (ср. version(none) тест в car.d —
    // «wheelWheel-отбраковка в fitness'е»).
    if (wheelAnchorSpacing(f) < wheelWheelMinGap)
        return 0.0f;

    // Число ведущих колёс и их разброс по сторонам; нижняя точка колёс.
    float xmin = float.max, xmax = -float.max;
    float ymin = float.max, ymax = -float.max;
    float groundZ = float.max;
    size_t nMotor = 0;
    foreach (a; f.anchors)
    {
        const vec3 p = f.nodes[a.node].pos;
        xmin = min(xmin, p.x); xmax = max(xmax, p.x);
        ymin = min(ymin, p.y); ymax = max(ymax, p.y);
        // Низ колеса — его центр минус генетический радиус.
        groundZ = min(groundZ, p.z - a.radius);
        if (a.kind == AnchorKind.motorWheel)
            nMotor += 1;
    }
    if (nMotor == 0)
        return 0.0f;

    // Опора — одно нижнее колесо: земля ложится по его нижней точке
    // (groundZ), все остальные колёса — где угодно: любой высоты, любого
    // места по бокам. Ничего ниже её огибающей каркасу нельзя.

    // Балки не должны зарываться ниже плоскости земли. Допуск epsFlat
    // прощает касание, но не проникновение.
    foreach (n; f.nodes)
        if (n.pos.z < groundZ - epsFlat)
            return 0.0f;

    // Кабина неприкосновенна: корпуса не касается ни одна балка и ни одно
    // колесо. Землю она тоже не касается: её низ (узел 0) — узел каркаса,
    // узлы же ниже опорной плоскости отброшены гейтом выше.
    if (frameCabinContact(f).length)
        return 0.0f;

    // ---- Слоты морфологии ----
    const float nodeSym = symmetryRatio(f);
    const float wheelSym = wheelSymmetry(f);
    const float pairSym = beamMassSymmetry(f);
    const float forkSym = forkRadiusSymmetry(ast);
    const float base = 0.5f * nodeSym + 0.3f * wheelSym + 0.2f * pairSym;
    const float phiSym = (0.5f + 0.5f * base) * forkSym;

    const size_t cycles = cyclomaticNumber(f); // μ = E - V + c
    const float phiRigid = 0.5f + 0.5f * (1.0f - exp(-0.4f * cast(float) cycles));

    const float phiDrive = 0.5f + 0.5f * motorBalance(f);

    // Габаритный ящик — единый потолок размера: параллелепипед
    // 3(курс) × 2.5(поперёк) × 2.5(высота над землёй), центр по колёсному
    // footprint. Доля узлов за его гранью гасит фитнес экспоненциально —
    // каркас не должен обрастать элементами за пределами габарита, а быть
    // меньше ящика — свободно.
    const float cx = 0.5f * (xmin + xmax);
    const float cy = 0.5f * (ymin + ymax);
    float outsideBox = 0;
    foreach (n; f.nodes)
    {
        const bool inBox = abs(n.pos.x - cx) <= boxWidthHalf + epsFlat
            && abs(n.pos.y - cy) <= boxCourseHalf + epsFlat
            && n.pos.z >= groundZ - epsFlat && n.pos.z <= groundZ + boxHeight + epsFlat;
        if (!inBox)
            outsideBox += 1.0f;
    }
    const float boxViolation = outsideBox / cast(float) f.nodes.length;
    const float phiBox = max(exp(-3.0f * boxViolation), morphologyFloor);

    return phiSym * phiRigid * phiDrive
        * phiBox;
}

/// Минимальный зазор между нижними ободами пар якорных колёс каркаса:
/// расстояние между центрами минус сумма генетических радиусов. Отрицателен
/// при перекрытии, нуль — колеса касаются друг друга.
float wheelAnchorSpacing(const Frame f)
{
    float best = float.max;
    foreach (i, a; f.anchors)
        foreach (j, b; f.anchors)
            if (j > i)
            {
                const float gap = distance(f.nodes[a.node].pos, f.nodes[b.node].pos)
                    - (a.radius + b.radius);
                best = min(best, gap);
            }
    return best;
}

/// Пол запретной зоны на курсе y (локальный, от ЦМ кабины): контур днища,
/// интерполяция хребта. За пределами станций — крайние значения контура.
private float cabinFloor(const CockpitGeometry cg, float yLocal)
{
    const spine = cg.spine;
    if (yLocal <= spine[0].y)
        return spine[0].z;
    if (yLocal >= spine[$ - 1].y)
        return spine[$ - 1].z;
    foreach (i; 1 .. spine.length)
        if (yLocal <= spine[i].y)
        {
            const t = (yLocal - spine[i - 1].y)
                / (spine[i].y - spine[i - 1].y);
            return spine[i - 1].z + (spine[i].z - spine[i - 1].z) * t;
        }
    assert(false);
}

/// Пересекает ли отрезок строгую внутренность зоны кабины: параллелепипед
/// `lo..hi` с полом по контуру днища (повторяет наклон). Балка в самой
/// плоскости пола зону не задевает — ловится только реальный проход сквозь
/// корпус.
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

    // Изломы пола (станции хребта) и границы окна — кандидаты на максимум
    // g(t) = z(t) − пол(y(t)); внутри каждого вдоль-линейного куска максимум
    // достигается на его концах.
    float[3 + 8] tPts;
    tPts[0] = tmin;
    size_t n = 1;
    if (abs(d.y) > 1e-12f)
        foreach (s; cg.spine)
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

/// Касается ли колесо кабины: центр диска внутри корпуса, расширенного на
/// радиус колеса; низ корпуса — по контуру днища на курсе колеса.
private bool wheelHitsCabin(const vec3 p, float r,
    const vec3 lo, const vec3 hi, const CockpitGeometry cg,
    const vec3 node0)
{
    if (!(p.x >= lo.x - r && p.x <= hi.x + r
        && p.y >= lo.y - r && p.y <= hi.y + r))
        return false;
    const float floorZ = node0.z + cabinFloor(cg, p.y - node0.y);
    return p.z + r > floorZ && p.z - r < hi.z;
}

/// Первое касание корпуса кабины в каркасе: пусто — никто её не трогает.
/// Запретная зона — параллелепипед от узла 0 (ЦМ кабины, совмещён с началом
/// координат меша) по AABB меша, пол по контуру днища (хребет). Эфемерные
/// балки крепления корпуса зону не проверяют: они не входят в каркас.
private string frameCabinContact(const Frame f)
{
    if (f.nodes.length == 0)
        return "";
    const cg = cockpitGeometry();
    const vec3 lo = f.nodes[0].pos + cg.minP;
    const vec3 hi = f.nodes[0].pos + cg.maxP;

    foreach (b; f.beams)
    {
        if (cast(Beam) b is null)
            continue;
        if (beamPiercesCabin(f.nodes[b.a].pos, f.nodes[b.b].pos,
            lo, hi, cg, f.nodes[0].pos))
            return "балка каркаса проходит сквозь кабину";
    }

    foreach (a; f.anchors)
        if (wheelHitsCabin(f.nodes[a.node].pos, a.radius, lo, hi, cg,
            f.nodes[0].pos))
            return "колесо заходит в кабину";

    return "";
}

enum double physicsDt = 1.0 / 60.0;
enum double physicsSimSeconds = 120.0; ///< окно заезда, 2 минуты
enum double physicsSettleSeconds = 1.0;
enum float physicsNominalSpeed = 4.0f;    ///< м/с фитнеса — дистанция-норма
enum float physicsSpeedCap = 2.0f;        ///< потолок бонуса за скорость доезда

struct PhysicsResult
{
    float score = 0.0f;
    bool survived = false;
    double distance = 0.0;
    double reachTime = 0.0;
    size_t wheels = 0;
    size_t beams = 0;
    string why = "";
}

/// Счёт — доля полной дистанции × средняя скорость доезда (в долях номинала).
PhysicsResult physicsFitness(const Buggy buggy, double seconds)
{
    PhysicsResult r;
    r.wheels = buggy.frame.anchors.length;

    if (buggy.frame.anchors.length < 2)
    {
        r.why = "объект с одним колесом";
        return r;
    }

    if (!canDrive(buggy.frame))
    {
        r.why = "нет привода";
        return r;
    }

    auto world = acquireWorld();
    scope (exit) releaseWorld(world);

    auto physics = new BuggyPhysics(buggy, world, sharedTerrain());
    scope (exit) physics.dispose();

    physics.settle(physicsDt,
        cast(int)(physicsSettleSeconds / physicsDt));

    const settleFailure = runFailure(physics);
    if (settleFailure.length)
    {
        r.why = settleFailure;
        return r;
    }

    // Кабина неприкосновенна и в заезде: рама — монолит, взаимоположение
    // кабины и каркаса не меняется, поэтому геометрия корпуса проверяется
    // один раз; живым остаётся только касание земли (см. cabinGroundContact).
    const cabinWhy = frameCabinContact(buggy.frame);
    if (cabinWhy.length)
    {
        r.why = cabinWhy;
        return r;
    }

    const size_t steps = cast(size_t)(seconds / physicsDt);

    auto wheels = physics.wheelStates();
    r.beams = physics.beamStates().length;

    double startY = 0.0;
    foreach (s; wheels)
        startY += s.position.y;
    startY /= wheels.length;

    double farthest = 0.0;
    double reachTime = seconds;
    foreach (i; 0 .. steps)
    {
        physics.step(physicsDt, 1.0f);

        const stepFailure = runFailure(physics);
        if (stepFailure.length)
        {
            r.why = stepFailure;
            break;
        }

        const cabinGround = physics.cabinTouchesGround();
        if (cabinGround)
        {
            r.why = "кабина касается земли";
            break;
        }

        wheels = physics.wheelStates();
        double curY = 0.0;
        foreach (s; wheels)
            curY += s.position.y;
        curY /= wheels.length;
        const double downhill = startY - curY;   // +Y — вверх по склону
        if (downhill > farthest)
        {
            farthest = downhill;
            reachTime = physicsDt * (i + 1);
        }
    }

    r.distance = farthest;
    r.reachTime = reachTime;
    // Зачёт — дожившим и остановившимся (переворот, застревание, кабина
    // коснулась земли) по пройденной дистанции; развалившийся каркас — 0.
    if (!structuralFailure(r.why))
        r.score = finishScore(farthest, seconds, reachTime);
    if (r.why.length != 0)
        return r;

    r.survived = true;
    return r;
}

/// Счёт заезда: доля полной дистанции × средняя скорость доезда (в долях номинала).
private float finishScore(double farthest, double seconds, double reachTime)
{
    const double distanceFrac = farthest / (physicsNominalSpeed * seconds);
    const double speedRatio = farthest / (physicsNominalSpeed * reachTime);
    return clamp(cast(float)(distanceFrac * speedRatio), 0.0f, physicsSpeedCap);
}

/// Разрушение каркаса: балки/колёса оборвались, ударились или ушли в землю.
/// Такие заезды засчёту не подлежат — машина развалилась, а не остановилась.
private bool structuralFailure(string why)
{
    return why == "не осталось колёс"
        || why == "каркас разлетелся"
        || why == "балка разлетелась"
        || why == "колесо провалилось под землю"
        || why == "балка каркаса касается земли"
        || why == "балка каркаса касается колеса"
        || why == "колёса каркаса соприкасаются";
}

unittest
{
    // Интактные остановки засчитываются, развал каркаса — нет.
    assert(structuralFailure("машина перевернулась") == false);
    assert(structuralFailure("нет продвижения вперёд") == false);
    assert(structuralFailure("кабина касается земли") == false);
    assert(structuralFailure("") == false);
    assert(structuralFailure("колёса каркаса соприкасаются"));
    assert(structuralFailure("не осталось колёс"));
    assert(structuralFailure("балка каркаса касается земли"));
}

unittest
{
    // finishScore: и общая дистанция, и скорость доезда влияют на счёт
    // мультипликативно; быстрее и дальше — выше.
    assert(finishScore(12.0, 3.0, 3.0) == 1.0f);
    assert(finishScore(6.0, 3.0, 3.0) == 0.25f);
    assert(finishScore(6.0, 3.0, 1.5) == 0.5f);
    assert(finishScore(12.0, 3.0, 1.5) == physicsSpeedCap);
    assert(finishScore(24.0, 3.0, 1.5) == physicsSpeedCap);
    assert(finishScore(0.0, 3.0, 3.0) == 0.0f);
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

    // Масса есть только у обычной балки: эфемерная не участвует.
    float mass(size_t i) {
        auto beam = cast(Beam) f.beams[i];
        if (beam is null)
            return 0.0f;
        const vec3 a = f.nodes[f.beams[i].a].pos;
        const vec3 c = f.nodes[f.beams[i].b].pos;
        return beam.radius * beam.radius * (c - a).length;
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
        // Эфемерная балка массы не имеет и пару обычной не занимает.
        const auto bi = cast(Beam) f.beams[i];
        if (bi is null)
            continue;

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
            const auto bj = cast(Beam) f.beams[j];
            if (bj is null)
                continue;
            if (abs(bj.radius - bi.radius) > 0.02f * bi.radius + 1e-4f)
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
    noWheels.nodes ~= Node(origin);
    noWheels.nodes ~= Node(right);
    noWheels.beams ~= new Beam(0, 1, 0.05f);
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
    split.beams ~= new Beam(2, 3, 0.04f);
    assert(!isConnected(split));
    assert(buggyFitness(split) == 0.0f);

    // Балки, торчащие ниже колёс: узел опускается под плоскость земли
    // (под нижнюю точку колёс) — физическая отбраковка. Земля на нижних
    // ободах: колёса на (−1.0..1.0, ±., −0.745), радиус 0.3 → z=−1.045.
    Frame underGround = symmetricBuggyFrame();
    underGround.nodes ~= Node(vec3(1.0f, 0.55f, -1.1f));
    underGround.beams ~= new Beam(2, underGround.nodes.length - 1, 0.04f);
    assert(buggyFitness(underGround) == 0.0f,
        "балка ниже уровня земли должна отбраковываться");

    // Тот же каркас с узлом, лишь касающимся земли (допуск), — не отбраковка.
    Frame boundary = symmetricBuggyFrame();
    boundary.nodes ~= Node(vec3(1.0f, 0.55f, -1.045f));
    boundary.beams ~= new Beam(2, boundary.nodes.length - 1, 0.04f);
    assert(buggyFitness(boundary) > 0.0f,
        "касание плоскости земли в пределах допуска не отбраковывается");

    // Симметричная машина должна оцениваться выше асимметричной той же формы.
    const float symFitness = buggyFitness(symmetricBuggyFrame());
    const float asymFitness = buggyFitness(asymmetricBuggyFrame());
    assert(symFitness > 0.0f && asymFitness > 0.0f,
        "обе машины физически выполнимы");
    assert(symFitness > asymFitness,
        "зеркальность колёс и каркаса даёт прирост фитнеса");

    // Балки за габаритом: идентичные каркасы, отличающиеся только одним
    // узлом (внутри ящика против выступающего наружу), — выступающий
    // получает меньший фитнес. Отличие — phiBox.
    Frame inside = symmetricBuggyFrame();
    inside.nodes ~= Node(vec3(0.5f, 0.3f, -0.9f));
    inside.beams ~= new Beam(2, inside.nodes.length - 1, 0.04f);
    Frame outside = symmetricBuggyFrame();
    outside.nodes ~= Node(vec3(1.6f, 0.3f, -0.9f));
    outside.beams ~= new Beam(2, outside.nodes.length - 1, 0.04f);
    assert(outside.nodes[$ - 1].pos.x > boxWidthHalf + epsFlat,
        "узел теста обязан выступать за полуширину ящика (1.25)");
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
        EndRef(EndRefKind.newNode, origin, 0),
        0.04f, 0.1f, 0.0f, BeamKind.normal, 0.0f);

    const f0 = buggyFitness(symmetricBuggyFrame(), a0);
    const fD = buggyFitness(symmetricBuggyFrame(), aD);
    assert(abs(f0 - base) < 1e-6f, "нулевой Nodal/Lefty не меняет фитнес");
    assert(fD < f0, "ненулевой |forkAsymmetry| штрафует асимметрию fork-пары");
}

unittest
{
    // Физический слой: заезд простейшего багги конечен, счёт не выше
    // потолка physicsSpeedCap и не зависит от статики. Пустой каркас — 0.
    const p = physicsFitness(new Buggy(placedFrame(symmetricBuggyFrame())), 1.0).score;
    assert(isFinite(p) && p >= 0.0f && p <= physicsSpeedCap,
        "счёт заезда нормирован и не разлетается");

    Frame empty;
    assert(physicsFitness(new Buggy(placedFrame(empty)), 1.0).score == 0.0f,
        "каркас без колёс не выезжает из нуля");
}

unittest
{
    // Кабина неприкосновенна: балка или колесо внутри корпуса отбраковывают
    // каркас, а крепёж ниже пола — нет.

    // Контроль: канонический багги кабину не трогает.
    assert(buggyFitness(symmetricBuggyFrame()) > 0.0f);

    // Балка из киля (под днищем) вертикально в корпус кабины — отбраковка.
    Frame pierce = symmetricBuggyFrame();
    pierce.nodes ~= Node(vec3(0.0f, 0.0f, 0.8f));
    pierce.beams ~= new Beam(1, pierce.nodes.length - 1, 0.04f);
    assert(frameCabinContact(pierce).length,
        "балка сквозь корпус кабины не должна проходить");
    assert(buggyFitness(pierce) == 0.0f);

    // Балка в плоскости пола кабины (граница зоны) — не касание.
    Frame mount = symmetricBuggyFrame();
    mount.nodes ~= Node(vec3(0.7f, 0.3f, -0.695f));
    mount.beams ~= new Beam(1, mount.nodes.length - 1, 0.04f);
    assert(frameCabinContact(mount).length == 0,
        "балка в плоскости пола не считается касанием");
    assert(buggyFitness(mount) > 0.0f);

    // Колесо, заходящее в корпус кабины, — отбраковка.
    Frame wheelInside = symmetricBuggyFrame();
    const wi = wheelInside.nodes.length; // добавить якорь в кабине
    wheelInside.nodes ~= Node(vec3(0.1f, 0.2f, 0.5f));
    wheelInside.anchors ~= Anchor(wi, AnchorKind.wheel);
    assert(frameCabinContact(wheelInside).length,
        "колесо внутри кабины не должно проходить");
    assert(buggyFitness(wheelInside) == 0.0f);
}

private Frame symmetricBuggyFrame()
{
    Frame f;
    size_t node(vec3 p)
    {
        f.nodes ~= Node(p);
        return f.nodes.length - 1;
    }

    const c = node(origin); // 0 — ЦМ кабины
    const k = node(vec3(0.0f, -0.205f, -0.745f)); // 1 — киль под днищем
    const fl = node(vec3(1.0f, 0.55f, -0.745f)); // 2
    const fr = node(vec3(-1.0f, 0.55f, -0.745f)); // 3
    const rl = node(vec3(1.0f, -0.35f, -0.745f)); // 4
    const rr = node(vec3(-1.0f, -0.35f, -0.745f)); // 5

    f.beams ~= new EphemeralBeam(c, k); // крепёж кабины не входит в каркас
    f.beams ~= new Beam(k, fl, 0.045f);
    f.beams ~= new Beam(k, fr, 0.045f);
    f.beams ~= new Beam(k, rl, 0.05f);
    f.beams ~= new Beam(k, rr, 0.05f);
    f.beams ~= new Beam(fl, fr, 0.045f); // передняя ось: петля
    f.beams ~= new Beam(rl, rr, 0.05f);  // задняя ось: петля

    f.anchors ~= Anchor(fl, AnchorKind.wheel);
    f.anchors ~= Anchor(fr, AnchorKind.wheel);
    f.anchors ~= Anchor(rl, AnchorKind.motorWheel);
    f.anchors ~= Anchor(rr, AnchorKind.motorWheel);
    f.motorPower = initialMotorPower;
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

    const c = node(origin); // 0
    const k = node(vec3(0.0f, -0.205f, -0.745f)); // 1
    const fl = node(vec3(1.0f, 0.55f, -0.745f)); // 2
    const fr = node(vec3(-1.0f, 0.55f, -0.745f)); // 3
    const rl = node(vec3(1.0f, -0.35f, -0.745f)); // 4

    f.beams ~= new EphemeralBeam(c, k);
    f.beams ~= new Beam(k, fl, 0.045f);
    f.beams ~= new Beam(k, fr, 0.045f);
    f.beams ~= new Beam(k, rl, 0.05f);
    f.beams ~= new Beam(fl, fr, 0.045f);

    f.anchors ~= Anchor(fl, AnchorKind.wheel);
    f.anchors ~= Anchor(fr, AnchorKind.wheel);
    f.anchors ~= Anchor(rl, AnchorKind.motorWheel);
    return f;
}
