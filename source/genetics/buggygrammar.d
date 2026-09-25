module genetics.buggygrammar;

import std.math;
import std.algorithm : min, max;
import std.typecons: Nullable;
import dlib.math.vector;
import frame.frame;
import frame.cockpit : cockpitGeometry;
import physics_world.wheel : defaultWheelRadius;
import genetics.sge;
import genetics.buggyast;

private Terminal!Tok t(T)(Tok tok)
{
    return new Terminal!Tok(tok);
}

private Terminal!Tok t(T)(Tok tok, T val)
{
    return new Terminal!Tok(tok, val);
}

private NonTerminal nt(string name, Production[] productions)
{
    auto n = new NonTerminal(name);
    n.productions = productions;
    return n;
}

/// Маркерный токен без значения
private Terminal!Tok marker(Tok tok)
{
    assert(tok == Tok.refLast || tok == Tok.refBase || tok == Tok.endNew
        || tok == Tok.endNear || tok == Tok.segStart || tok == Tok.fork
        || tok == Tok.anchors);

    return new Terminal!Tok(tok);
}

/**
 * Грамматика багги целиком.
 *
 * Каркас — последовательность сегментов, модульных групп балок. Каждый
 * сегмент начинается маркером `segStart`, за ним идёт необязательный
 * `Tok.fork` (раздвоение). Асимметрия каждой балки пары задаётся токенами
 * `nodal` (активатор) и `lefty` (ингибитор). Узлы отдельно не генерируются:
 * они появляются только как концы балок.
 * Каждая балка: старт — первый/последний созданный узел или существующий
 * по индексу, конец — вновь создаваемый (старт + смещение) или существующий
 * по индексу.
 *
 * Якоря генерируются после всех балок: это пара `anchorKind` + `refIdx`,
 * где `refIdx` ссылается на уже созданный узел.
 */
Grammar buggyGrammar()
{
    auto segmentList_ = nt("segmentList", null);
    auto beamList_ = nt("beamList", null);

    // Раздвоение сегмента: пустая продукция — медианная одиночная структура
    // («глаз по центру»), маркер `Tok.fork` — пара ветвей вокруг локальной
    // оси (X стартового узла сегмента). Асимметрия пары задаётся не статикой
    // всего сегмента, а парой активатор-ингибитор `nodal`/`lefty` на каждой
    // балке — Twin-балка масштабируется в `1 + forkAsymmetry(nodal, lefty)`,
    // геометрия остаётся зеркальной (как Nodal/Lefty у позвоночных).
    auto segMode = nt("segMode", [
        new Production([]),
        new Production([marker(Tok.fork)]),
    ]);

    auto nodal = new Sampler!Tok("nodal", Tok.nodal, -0.2f, 0.2f);
    auto lefty = new Sampler!Tok("lefty", Tok.lefty, 0.0f, 8.0f);

    auto segment = nt("segment", [
        new Production([marker(Tok.segStart), segMode, beamList_]),
    ]);

    segmentList_.productions = [
        new Production([segment, segmentList_]),
        new Production([segment]),
    ];

    auto idx = new Sampler!Tok("idx", Tok.refIdx, 64);
    // Направления turtle: дельта «вперёд» идёт по заголовку, «вправо» —
    // перпендикулярно от него, «вверх» — вертикально. Три токена всё так же
    // `Tok.coord`, но гены называются по смыслу, а не по осям X/Y/Z.
    auto forward = new Sampler!Tok("forward", Tok.coord, -1.5f, 1.5f);
    auto right = new Sampler!Tok("right", Tok.coord, -1.5f, 1.5f);
    auto up = new Sampler!Tok("up", Tok.coord, -1.5f, 1.5f);
    auto startForward = new Sampler!Tok("startForward", Tok.coord, -2.0f, 2.0f);
    auto startRight = new Sampler!Tok("startRight", Tok.coord, -2.0f, 2.0f);
    auto startUp = new Sampler!Tok("startUp", Tok.coord, -2.0f, 2.0f);
    auto radius = new Sampler!Tok("radius", Tok.radius, 0.02f, 0.06f);
    auto taper = new Sampler!Tok("taper", Tok.taper, 0.4f, 1.0f);
    auto taperPow = new Sampler!Tok("taperPow", Tok.taperPow, 0.5f, 4.0f);
    auto heading = new Sampler!Tok("heading", Tok.heading, -3.1416f, 3.1416f);
    auto turn = new Sampler!Tok("turn", Tok.turn, -1.5708f, 1.5708f);
    auto motorPower = new Sampler!Tok("motorPower", Tok.motorPower, -200.0f, 200.0f);

    auto startRef = nt("startRef", [
        new Production([marker(Tok.refLast)]),
        new Production([marker(Tok.refBase)]),
        new Production([idx]),
    ]);

    auto startPos = nt("startPos", [
        new Production([startForward, startRight, startUp, taper, taperPow, heading, motorPower]),
    ]);

    auto endRef = nt("endRef", [
        new Production([marker(Tok.endNew), forward, right, up]),
        new Production([idx]),
        new Production([marker(Tok.endNear), forward, right, up]),
    ]);

    auto beamKind = nt("beamKind", [
        new Production([t(Tok.beamKind, BeamKind.normal)]),
    ]);

    auto beam = nt("beam", [
        new Production([startRef, endRef, radius, nodal, lefty, beamKind, turn]),
    ]);

    beamList_.productions = [
        new Production([beam, beamList_]),
        new Production([beam]),
    ];

    auto anchorList_ = nt("anchorList", null);

    auto anchorKind = nt("anchorKind", [
        new Production([t(Tok.anchorKind, AnchorKind.wheel)]),
        new Production([t(Tok.anchorKind, AnchorKind.motorWheel)]),
    ]);

    // Радиус колеса якоря: свой у каждого якоря.
    auto wheelRadius = new Sampler!Tok("wheelRadius", Tok.wheelRadius, 0.05f, 0.375f);

    auto anchor = nt("anchor", [
        new Production([anchorKind, idx, wheelRadius]),
    ]);

    anchorList_.productions = [
        new Production([anchor, anchorList_]),
        new Production([anchor]),
    ];

    // Маркер конца балок и начала якорей.
    auto anchorMarker = nt("anchorMarker", [
        new Production([new Terminal!Tok(Tok.anchors)]),
    ]);

    auto start = nt("frame", [
        new Production([startPos, segmentList_, anchorMarker, anchorList_]),
    ]);

    auto symbols = [
        start, startPos, startForward, startRight, startUp, taper, taperPow, heading, turn,
        motorPower,
        segmentList_, segment, segMode, nodal, lefty,
        beamList_, beam, startRef,
        idx, endRef, forward, right, up, radius, beamKind,
        anchorMarker, anchorList_, anchor, anchorKind, wheelRadius,
    ];
    return new Grammar(start, symbols);
}

/// Вес гена в точечной мутации: правки попадают в него с этой долей.
float[] mutationWeights(const Grammar gr)
{
    auto w = new float[gr.symbols.length];
    foreach (i, sym; gr.symbols)
        w[i] = geneMutWeight(sym.name);
    return w;
}

/// Вес i-го гена: 0 — не мишень вовсе, 1 — обычная.
private float geneMutWeight(string name)
{
    // Значения-индексы узлов: флип почти всегда ссылка на несуществующий узел.
    if (name == "idx")
        return 0.0f;
    // Списки элементов: точечная правка переключает «продолжить/конец»,
    // обрывая или раздувая цепочку — для этого есть инделы.
    if (name == "beamList" || name == "segmentList" || name == "anchorList")
        return 0.2f;
    // Ссылка на узел: refLast/refBase безопасны, idx-ветвь — не всегда.
    if (name == "startRef" || name == "endRef")
        return 0.5f;
    // Выбор единственной продукции: флип не меняет фенотип.
    if (name == "frame" || name == "startPos" || name == "segment"
        || name == "beam" || name == "beamKind" || name == "anchor"
        || name == "anchorMarker")
        return 0.0f;
    return 1.0f;
}

unittest
{
    // Вес на каждый ген грамматики; токсичный idx — не мишень, а геометрия
    // и структурные списки получают обычный и пониженный веса.
    auto gr = buggyGrammar();
    auto w = mutationWeights(gr);
    assert(w.length == gr.symbols.length, "вес на каждый ген");

    auto idOf = (string n) {
        foreach (sym; gr.symbols)
            if (sym.name == n)
                return sym.id;
        assert(false);
    };

    assert(w[idOf("idx")] == 0.0f, "idx не мишень точечной мутации");
    assert(w[idOf("forward")] == 1.0f, "геометрия — обычная мишень");
    assert(w[idOf("beamList")] < 1.0f, "структура растёт инделами");
    assert(w[idOf("startRef")] < 1.0f, "ссылки мутируются осторожно");
}

/**
 * Построить геометрию каркаса из AST.
 *
 * Исполнитель «глупо» выполняет решения дерева: читает turtle-заголовок,
 * разрешает рефы, при раздвоенном сегменте рождает узлы парой вокруг оси
 * сегмента (X его стартового узла), масштабирует радиусы морфоген-градиентом
 * и дублирует балки/якоря в twin. Вся семантика — fork, ось, морфоген,
 * turtle — зафиксирована в AST.
 */
Nullable!Frame frameFromAst(const Ast ast)
{
    enum float mergeRadius = 0.15f;

    Frame result;

    // Таблица пар: node -> его twin вокруг оси раздвоения сегмента.
    size_t[] forkOf;

    // Узел; при раздвоении сегмента — пара (узел, twin) вокруг оси.
    auto addNode = (vec3 target, bool fork, float axis) {
        result.nodes ~= Node(target);
        const n = result.nodes.length - 1;
        if (fork && abs(target.x - axis) >= 1e-4f)
        {
            result.nodes ~= Node(vec3(2.0f * axis - target.x, target.y, target.z));
            const nm = result.nodes.length - 1;
            forkOf.length = result.nodes.length;
            forkOf[n] = nm;
            forkOf[nm] = n;
        }
        else
        {
            forkOf.length = result.nodes.length;
            forkOf[n] = n;
        }
        return n;
    };

    size_t last = addNode(origin, false, 0.0f);

    // Скелет крепления: станции «хребта» и боковые пары на общей плоскости
    // ниже днища (кель), эфемерные балки между ними. Нода 0 — ЦМ кабины.
    auto insertSkeleton = () {
        const auto cg = cockpitGeometry();

        const size_t spine0 = result.nodes.length;
        foreach (sp; cg.spine)
            addNode(sp, false, 0.0f);
        foreach (pair; cg.spineSides)
            addNode(pair[0], true, 0.0f);

        void eph(size_t a, size_t b)
        {
            result.beams ~= new EphemeralBeam(a, b);
        }

        // От ЦМ к ближайшей станции и цепь по всему хребту.
        size_t best = 0;
        float bestD = distance(origin, cg.spine[0]);
        foreach (i; 1 .. cg.spine.length)
        {
            const d = distance(origin, cg.spine[i]);
            if (d < bestD)
            {
                bestD = d;
                best = i;
            }
        }
        eph(0, spine0 + best);
        foreach (i; 0 .. cg.spine.length - 1)
            eph(spine0 + i, spine0 + i + 1);

        // Каждая боковая пара крепится балками к своей станции.
        foreach (j; 0 .. cg.spineSides.length)
        {
            const size_t right = result.nodes.length - 2 * (cg.spineSides.length - j);
            const size_t left = right + 1;
            eph(spine0 + cg.spineSideAt[j], right);
            eph(spine0 + cg.spineSideAt[j], left);
        }
        return result.nodes.length - 1;
    };
    last = insertSkeleton();

    float heading = ast.heading;
    auto turtleDelta = (float dx, float dy, float dz) {
        const c = cos(heading);
        const s = sin(heading);
        return vec3(c * dx - s * dy, s * dx + c * dy, dz);
    };

    size_t nBeams;
    foreach (seg; ast.segments)
        nBeams += seg.beams.length;

    size_t j;
    foreach (seg; ast.segments)
    {
        // «Зачаток» сегмента: узел, на котором начался сегмент. `refBase`
        // возвращает рост к нему — так из одной точки ветвления выпускаются
        // несколько отростков (пальцы из «запястья») без точных индексов.
        const segBase = last;

        bool haveAxis = false;
        float axis = 0.0f;

        foreach (b; seg.beams)
        {
            size_t start;
            final switch (b.start.kind)
            {
                case StartRefKind.last:
                    start = last;
                    break;
                case StartRefKind.base:
                    start = segBase;
                    if (start >= result.nodes.length)
                        return Nullable!Frame.init;
                    break;
                case StartRefKind.idx:
                    start = b.start.idx;
                    if (start >= result.nodes.length)
                        return Nullable!Frame.init;
                    break;
            }

            if (seg.fork && !haveAxis)
            {
                // Наследование оси: если начало сегмента — член пары из
                // предыдущего раздвоения, ось — середина пары (локальная
                // плоскость симметрии структуры), а не «родин»X одной стороны.
                if (forkOf[start] != start)
                    axis = 0.5f * (result.nodes[start].pos.x
                        + result.nodes[forkOf[start]].pos.x);
                else
                    axis = result.nodes[start].pos.x;
                haveAxis = true;
            }

            size_t end;
            final switch (b.end.kind)
            {
                case EndRefKind.newNode:
                {
                    auto target = result.nodes[start].pos
                        + turtleDelta(b.end.delta.x, b.end.delta.y, b.end.delta.z);
                    end = addNode(target, seg.fork, axis);
                    last = end;
                    break;
                }
                case EndRefKind.nearNode:
                {
                    auto target = result.nodes[start].pos
                        + turtleDelta(b.end.delta.x, b.end.delta.y, b.end.delta.z);
                    // Растущий конец сливается с ближайшим существующим узлом
                    // (кроме старта) в пределах mergeRadius — так сами возникают
                    // петли и самосборка каркаса.
                    size_t best = size_t.max;
                    auto bestD = mergeRadius;
                    foreach (n; 0 .. result.nodes.length)
                    {
                        if (n == start)
                            continue;
                        const d = distance(result.nodes[n].pos, target);
                        if (d < bestD)
                        {
                            bestD = d;
                            best = n;
                        }
                    }
                    if (best != size_t.max)
                        end = best;
                    else
                        end = addNode(target, seg.fork, axis);
                    last = end;
                    break;
                }
                case EndRefKind.idx:
                    end = b.end.idx;
                    if (end >= result.nodes.length)
                        return Nullable!Frame.init;
                    break;
            }

            // Морфоген-градиент толщины: порядок построения балки -> радиус.
            float radius = b.radius;
            if (nBeams >= 2)
            {
                const frac = cast(float) j / (nBeams - 1);
                radius *= 1.0f + (ast.taper - 1.0f) * pow(frac, ast.taperPow);
            }
            ++j;

            // Балка, касающаяся узла 0 (ЦМ кабины), — эфемерная: крепление
            // через скелет, а не сквозь кабину. Нода 0 — точка роста, без
            // массы и рендера.
            const bool onAxis = start == 0 || end == 0;
            result.beams ~= addEvolvedBeam(start, end, radius, b.kind, onAxis);
            if (seg.fork && !(forkOf[start] == start && forkOf[end] == end))
                result.beams ~= addEvolvedBeam(forkOf[start], forkOf[end],
                    radius * (1.0f + forkAsymmetry(b.nodal, b.lefty)), b.kind,
                    onAxis || forkOf[start] == 0 || forkOf[end] == 0);

            heading += b.turn;
        }
    }

    foreach (a; ast.anchors)
    {
        auto n = cast(size_t) a.idx % result.nodes.length;
        result.anchors ~= Anchor(n, a.kind, a.radius);
        if (forkOf[n] != n)
            result.anchors ~= Anchor(forkOf[n], a.kind, a.radius);
    }
    result.motorPower = ast.motorPower;
    return Nullable!Frame(result);
}

/// Балка роста: при касании узла 0 (ЦМ) — эфемерная, иначе обычная.
private EphemeralBeam addEvolvedBeam(size_t a, size_t b, float radius,
    BeamKind kind, bool ephemeral)
{
    if (ephemeral)
        return new EphemeralBeam(a, b);
    return new Beam(a, b, radius, kind);
}

/// Число узлов скелета крепления (нода 0 + хребет + пары).
size_t skeletonNodeCount()
{
    const auto cg = cockpitGeometry();
    return 1 + cg.spine.length + cg.spineSides.length * 2;
}

/// Число балок скелета (эфемерных) — для тестов и потребителей.
size_t skeletonBeamCount()
{
    const auto cg = cockpitGeometry();
    return 1 + (cg.spine.length - 1) + cg.spineSides.length * 2;
}

enum float minBeamLength = 0.05f;
enum float maxBeamLength = 3.0f;

/// Допуск совпадения отрезков в теле: зазор меньше, чем реальные радиусы,
/// поэтому ловит проход через одну точку, а не близкий пролёт.
enum float beamCrossGap = 1e-3f;

/// Пересечение двух отрезков в их строгой внутренности. Общий конец (стык
/// балок в одном узле) пересечением не считается.
private bool beamsCross(const vec3 a1, const vec3 a2, const vec3 b1, const vec3 b2)
{
    const vec3 d1 = a2 - a1;
    const vec3 d2 = b2 - b1;
    const vec3 r = b1 - a1;

    const float a = d1.lengthsqr;
    const float e = d2.lengthsqr;
    const float b = dot(d1, d2);
    const float denom = a * e - b * b; // |d1 × d2|²
    if (denom <= 1e-12f)
        return false; // коллинеарные (твины форка) не пересекаются в точке
    const float c = dot(d1, r);
    const float f = dot(d2, r);
    const float s = (e * c - b * f) / denom;
    const float t = (b * c - a * f) / denom;
    if (s <= 0.0f || s >= 1.0f || t <= 0.0f || t >= 1.0f)
        return false;
    return ((a1 + d1 * s) - (b1 + d2 * t)).lengthsqr
        < beamCrossGap * beamCrossGap;
}

/**
 * Структурная валидность каркаса: узлы в границах, без вырожденных балок,
 * все балки в допустимом диапазоне длин, обычные балки не пересекаются в
 * теле, якоря — на валидных узлах, каркас связен.
 */
Nullable!Frame isValidFrame(Frame f)
{
    if (f.nodes.length == 0 || f.beams.length == 0)
        return Nullable!Frame.init;
    foreach (b; f.beams)
    {
        if (b.a >= f.nodes.length || b.b >= f.nodes.length)
            return Nullable!Frame.init;
        if (b.a == b.b)
            return Nullable!Frame.init;
        const len = distance(f.nodes[b.a].pos, f.nodes[b.b].pos);
        if (len < minBeamLength || len > maxBeamLength)
            return Nullable!Frame.init;
    }

    // Эфемерные балки скелета в проверке не участвуют: они не несут тела.
    foreach (i; 0 .. f.beams.length)
    {
        if (cast(Beam) f.beams[i] is null)
            continue;
        foreach (j; i + 1 .. f.beams.length)
        {
            if (cast(Beam) f.beams[j] is null)
                continue;
            if (beamsCross(f.nodes[f.beams[i].a].pos, f.nodes[f.beams[i].b].pos,
                f.nodes[f.beams[j].a].pos, f.nodes[f.beams[j].b].pos))
                return Nullable!Frame.init;
        }
    }

    // Дубли якорей на одном узле допускаются — это вырожденный случай, который
    // отсеется на этапе физики/фитнеса, а не на этапе синтаксиса.
    foreach (a; f.anchors)
        if (a.node >= f.nodes.length)
            return Nullable!Frame.init;

    if (!isConnected(f))
        return Nullable!Frame.init;
    return Nullable!Frame(f);
}

struct Developed
{
    Frame frame;
    Ast ast;
}

/// Расшифровать геном из грамматики багги в каркас вместе с его AST.
/// `Nullable!Developed.isNull` - признак неудачи
Nullable!Developed develop(const Grammar gr, const Genotype g)
{
    auto tokens = decode!Tok(gr, g);
    if (tokens is null)
        return Nullable!Developed.init;
    auto mayAst = buildAst(tokens);
    if (mayAst.isNull)
        return Nullable!Developed.init;
    auto frame = frameFromAst(mayAst.get);
    if (frame.isNull)
        return Nullable!Developed.init;
    auto valid = isValidFrame(frame.get);
    if (valid.isNull)
        return Nullable!Developed.init;
    return Nullable!Developed(Developed(valid.get, mayAst.get));
}

/// Токены -> AST -> геометрия (для тестов и пробников).
Nullable!Frame toFrame(const Terminal!Tok[] tokens)
{
    auto ast = buildAst(tokens);
    if (ast.isNull)
        return Nullable!Frame.init;
    return frameFromAst(ast.get);
}

version (unittest)
{
    /// Дельта конца балки или начальное смещение как «единичный вектор
    /// направления × множитель длины» (для тестов): возвращает три токена
    /// `coord` — вперёд, вправо, вверх.
    Terminal!Tok[] dirCoords(vec3 dir, float len)
    {
        const v = dir * len;
        return [
            new Terminal!Tok(Tok.coord, v.x),
            new Terminal!Tok(Tok.coord, v.y),
            new Terminal!Tok(Tok.coord, v.z),
        ];
    }
}

unittest
{
    import std.random : Random;

    auto gr = buggyGrammar();
    auto rnd = Random(42);

    size_t valid, decodeOk;
    foreach (_; 0 .. 5000)
    {
        auto g = randomGenotype(gr, 8, rnd);
        assert(g.genes.length == gr.symbols.length);

        auto tokens = decode!Tok(gr, g);
        if (tokens is null)
            continue;
        ++decodeOk;
        assert(tokens.length > 0);

        auto f = toFrame(tokens);
        if (f.isNull)
            continue;

        if (!isValidFrame(f.get).isNull)
            ++valid;
    }

    // Декодирование сходится почти всегда; значительная доля геномов даёт
    // валидный каркас (остальные отбрасываются самокоррекцией — ссылки на
    // узлы, которых ещё нет, вырожденные балки).
    assert(decodeOk > 3000);
    assert(valid > 20);

    // Кроссинговер сохраняет число генов (по одному на нетерминал).
    auto g1 = randomGenotype(gr, 8, rnd);
    auto g2 = randomGenotype(gr, 8, rnd);
    auto child = crossover(g1, g2, rnd);
    assert(child.genes.length == gr.symbols.length);
}

unittest
{
    // endNear: растущий конец сливается с ближайшим существующим узлом
    // в пределах допуска — возникает замкнутая петля без нового узла.
    Terminal!Tok[] t;
    // Seed: начало координат, смещение нулевое.
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    // Первая балка: от конца скелета (узел 11) к (0, 0.6, 0).
    t ~= dirCoords(vec3(0.35f, -0.095f, 0.499f), 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    // Вторая балка: назад на 0.6 — конец точно в узле 0 (ЦМ).
    t ~= dirCoords(vec3(0.0f, -0.6f, 0.0f), 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == skeletonNodeCount() + 1,
        "endNear должен слиться с узлом 0, а не создавать новый");
    assert(f.get.beams.length == skeletonBeamCount() + 2);
    assert(f.get.beams[skeletonBeamCount() + 1].a == skeletonNodeCount()
        && f.get.beams[skeletonBeamCount() + 1].b == 0,
        "вторая балка замыкает петлю на узел 0 (ЦМ)");
    assert(cast(Beam) f.get.beams[skeletonBeamCount() + 1] is null,
        "балка, касающаяся узла 0, — эфемерная, без массы");
    assert(cast(Beam) f.get.beams[skeletonBeamCount()] !is null,
        "первая балка не касается ЦМ — обычная");
    assert(!isValidFrame(f.get).isNull, "замкнутая петля остаётся связной");
}

unittest
{
    // endNear далеко от структуры ведёт себя как endNew — новый узел.
    Terminal!Tok[] t;
    // Seed: начало координат.
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    // Растущий конец далеко от структуры: вверх на 1 (над килем никого нет).
    t ~= dirCoords(up, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == skeletonNodeCount() + 1);
    assert(f.get.beams.length == skeletonBeamCount() + 1);
}

unittest
{
    // Turtle: дельты заданы в базисе каркаса, а heading (и кумулятивный turn)
    // поворачивает их в плоскости X-Y: x' = c·dx − s·dy, y' = s·dx + c·dy.
    // Проверяем проекции на направления рамы, а не на абсолютные оси.
    Terminal!Tok[] t;
    // Seed: начало координат.
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 1.5707963f);
    t ~= new Terminal!Tok(Tok.segStart);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    // Дельты — единичные векторы в базисе каркаса: вперёд×1, затем вправо×1.
    t ~= dirCoords(forward, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= dirCoords(right, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == skeletonNodeCount() + 2);
    const n1 = f.get.nodes[skeletonNodeCount()].pos;
    const n2 = f.get.nodes[skeletonNodeCount() + 1].pos;
    // Первая балка растёт от узла 11 (конец скелета) — сдвиг от него.
    const vec3 d1 = n1 - f.get.nodes[11].pos;
    // Заголовок π/2 поворачивает дельту вперёд вдоль right-направления рамы.
    assert(dot(d1, right) > 0.999f,
        "заголовок π/2 должен развернуть дельту вперёд вдоль right");
    assert(abs(dot(d1, forward)) < 1e-4f && abs(dot(d1, up)) < 1e-4f,
        "поворот не уводит дельту наружу плоскости рамы");
    // Дельту right заголовок π/2 поворачивает вдоль backward; узлы копятся
    // от предыдущей точки, а не от начала координат.
    const vec3 d2 = n2 - n1;
    assert(dot(d2, backward) > 0.999f,
        "дельту right при заголовке π/2 поворачивает вдоль backward");
    assert(abs(dot(d2, right)) < 1e-4f && abs(dot(d2, up)) < 1e-4f,
        "вторая дельта тоже лежит в плоскости рамы");
}

unittest
{
    // Раздвоение (fork): сегмент, стартующий с узла боковой пары скелета
    // (11 — левый борт кормы), наследует ось сегмента из середины пары и
    // продолжает каждую балку зеркальной парой. Твин-балка получает
    // radius·(1+nodal).
    Terminal!Tok[] t;
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    // Ствол: вперёд×1; ветвь: вправо×1.
    t ~= dirCoords(forward, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.1f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= dirCoords(right, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.1f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.wheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) skeletonNodeCount());

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == skeletonNodeCount() + 4,
        "ствол-пара + пара ветвей");
    const vec3 n1 = f.get.nodes[skeletonNodeCount()].pos;
    const vec3 n2 = f.get.nodes[skeletonNodeCount() + 1].pos;
    const vec3 n3 = f.get.nodes[skeletonNodeCount() + 2].pos;
    const vec3 n4 = f.get.nodes[skeletonNodeCount() + 3].pos;
    assert(abs(n1.x + 0.35f) < 1e-4f && abs(n1.y + 0.305f) < 1e-4f,
        "ствол продолжает левый борт кормы: (-0.35,-0.305)");
    assert(abs(n2.x - 0.35f) < 1e-4f && abs(n2.y - n1.y) < 1e-4f,
        "твин ствола зеркалит через ось сегмента");
    assert(abs(n3.x - 0.65f) < 1e-4f,
        "ветвь идёт вправо от конца ствола: (0.65,-0.305)");
    assert(abs(n4.x + 0.65f) < 1e-4f,
        "твин-ветвь зеркальна вокруг оси: (-0.65,-0.305)");

    assert(f.get.beams.length == skeletonBeamCount() + 4);
    assert(f.get.beams[skeletonBeamCount()].a == 11
        && f.get.beams[skeletonBeamCount()].b == skeletonNodeCount());
    assert(f.get.beams[skeletonBeamCount() + 1].a == 10
        && f.get.beams[skeletonBeamCount() + 1].b == skeletonNodeCount() + 1,
        "зеркальная половина ствола — от правого борта");
    assert(f.get.beams[skeletonBeamCount() + 2].a == skeletonNodeCount()
        && f.get.beams[skeletonBeamCount() + 2].b == skeletonNodeCount() + 2,
        "вторая балка — ветвь из конца ствола");
    assert(f.get.beams[skeletonBeamCount() + 3].a == skeletonNodeCount() + 1
        && f.get.beams[skeletonBeamCount() + 3].b == skeletonNodeCount() + 3,
        "третья — твин ветви через ось сегмента");

    // Nodal=0.1 без ингибитора даёт сдвиг 0.1; радиус твин-балки:
    // 0.04 · (1 + 0.1) = 0.044.
    assert(abs(asBeam(f.get.beams[skeletonBeamCount()]).radius - 0.04f) < 1e-4f);
    assert(abs(asBeam(f.get.beams[skeletonBeamCount() + 3]).radius - 0.044f) < 1e-4f);

    assert(f.get.anchors.length == 2, "ствол-пара несёт пару якорей");
    assert(f.get.anchors[0].node == skeletonNodeCount()
        && f.get.anchors[1].node == skeletonNodeCount() + 1);

    assert(!isValidFrame(f.get).isNull,
        "раздвоенный каркас со стволом от боковой пары остаётся связным");
}

unittest
{
    // Раздвоение поверх turtle: twin-половина поворачивается вместе
    // с заголовком. Заголовок π/4 раскладывает дельту вперёд на направления
    // рамы: right·s + forward·c; twin одерживает right-составляющую.
    Terminal!Tok[] t;
    // Seed: начало координат — ось сегмента X == 0.
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.78539815f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    // Ветвь — единичный вектор вперёд×1, twin вокруг оси сегмента.
    t ~= dirCoords(forward, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == skeletonNodeCount() + 2);
    const vec3 n1 = f.get.nodes[skeletonNodeCount()].pos;
    const vec3 n2 = f.get.nodes[skeletonNodeCount() + 1].pos;
    const float c = cos(0.78539815f);
    const float s = sin(0.78539815f);
    // Дельта вперёд при заголовке π/4: right·s + forward·c.
    const vec3 d1 = n1 - f.get.nodes[11].pos;
    assert(abs(dot(d1, right) - s) < 1e-4f
        && abs(dot(d1, forward) - c) < 1e-4f,
        "дельта вперёд при заголовке π/4 ложится на right·s + forward·c");
    assert(abs(dot(d1, up)) < 1e-4f, "поворот плоский, из рамы не уводит");
    // Twin зеркалит right-составляющую вокруг оси сегмента.
    const vec3 d2 = n2 - f.get.nodes[10].pos;
    assert(abs(dot(d2, right) + s) < 1e-4f
        && abs(dot(d2, forward) - c) < 1e-4f,
        "twin одерживает right-составляющую, forward не трогается");
}

unittest
{
    // Локальная ось раздвоения: сегмент, начатый с узла боковой пары
    // (refIdx 6 — правый передний борт), наследует ось из середины пары,
    // а не X стартового узла. Ветви зеркалят вокруг середины пары.
    Terminal!Tok[] t;
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);

    t ~= new Terminal!Tok(Tok.refIdx, cast(int) 6);
    t ~= new Terminal!Tok(Tok.endNew);
    // Ствол: вперёд×1; ветвь: вправо×1.
    t ~= dirCoords(forward, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= dirCoords(right, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == skeletonNodeCount() + 4,
        "ствол-пара + пара ветвей от боковой станции");
    assert(abs(f.get.nodes[skeletonNodeCount()].pos.x - 0.30f) < 1e-4f
        && abs(f.get.nodes[skeletonNodeCount()].pos.y + 2.405f) < 1e-4f,
        "ствол идёт вперёд от правого переднего борта");
    assert(abs(f.get.nodes[skeletonNodeCount() + 1].pos.x + 0.30f) < 1e-4f,
        "твин ствола — от левого борта той же станции");
    const float yBranch = f.get.nodes[skeletonNodeCount() + 1].pos.y;
    assert(abs(f.get.nodes[skeletonNodeCount() + 2].pos.x - 1.30f) < 1e-4f
        && abs(f.get.nodes[skeletonNodeCount() + 2].pos.y - yBranch) < 1e-4f,
        "ветвь растёт вправо от конца ствола");
    assert(abs(f.get.nodes[skeletonNodeCount() + 3].pos.x + 1.30f) < 1e-4f,
        "twin ветви зеркалит вокруг оси — середины пары, а не узла старта");
    assert(!isValidFrame(f.get).isNull,
        "обе стороны связаны парами боковой станции — каркас связен");
}

unittest
{
    // Сегмент без раздвоения — медианная одиночная структура без twin:
    // ни один узел не дублируется.
    Terminal!Tok[] t;
    // Seed: начало координат.
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    // Одиночная балка: направление (1 вперёд, 1 вправо) × 1.
    t ~= dirCoords(vec3(1.0f, 1.0f, 0.0f), 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.motorWheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) skeletonNodeCount());

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == skeletonNodeCount() + 1);
    assert(f.get.beams.length == skeletonBeamCount() + 1);
    assert(cast(Beam) f.get.beams[skeletonBeamCount()] !is null,
        "обычная балка без раздвоения");
    assert(f.get.anchors.length == 1,
        "без раздвоения якорь не дублируется");
    assert(f.get.anchors[0].node == skeletonNodeCount());
    assert(abs(f.get.anchors[0].radius - defaultWheelRadius) < 1e-6f,
        "вручную собранный поток без wheelRadius даёт якорь заводского размера");
}

unittest
{
    // Генетический радиус колеса проходит от токена через AST в якорь,
    // включая twin-якорь раздвоенного сегмента — оба колеса fork-пары
    // получают один размер.
    Terminal!Tok[] t;
    // Seed: начало координат — ось сегмента X == 0.
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    // Ствол: вперёд×1 (по оси, без twin); ветвь: вправо×1 (с twin).
    t ~= dirCoords(forward, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= dirCoords(right, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.wheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) skeletonNodeCount());
    t ~= new Terminal!Tok(Tok.wheelRadius, 0.37f);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.anchors.length == 2, "twin-якорь дублируется вместе с узлом");
    assert(f.get.anchors[0].node == skeletonNodeCount()
        && f.get.anchors[1].node == skeletonNodeCount() + 1,
        "колёса висят на концах пары");
    assert(abs(f.get.anchors[0].radius - 0.37f) < 1e-6f
        && abs(f.get.anchors[1].radius - 0.37f) < 1e-6f,
        "радиус несётся в оба якоря fork-пары");
}

unittest
{
    // refBase — «зачаток»: балка стартует с базового узла сегмента (конец
    // скелета, узел 11), а не с последнего. Первая балка — ствол форка,
    // из базы выпускаются отростки-«пальцы», каждый в своей зеркальной паре.
    Terminal!Tok[] t;
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    // Рука: вперёд×1.
    t ~= dirCoords(forward, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refBase);
    t ~= new Terminal!Tok(Tok.endNew);
    // Первый палец: направление (0.5 вперёд, 1 вправо) × 1.
    t ~= dirCoords(vec3(0.5f, 1.0f, 0.0f), 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refBase);
    t ~= new Terminal!Tok(Tok.endNew);
    // Второй палец: направление (1.5 вперёд, 1 вправо) × 1.
    t ~= dirCoords(vec3(1.5f, 1.0f, 0.0f), 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    // Скелет + ствол-пара + две пальцевые пары.
    assert(f.get.nodes.length == skeletonNodeCount() + 6);
    assert(f.get.beams.length == skeletonBeamCount() + 6,
        "ствол + по паре на каждый палец");

    // Ствол: рука от базы (узел 11 — конец скелета) с зеркальной парой.
    assert(f.get.beams[skeletonBeamCount()].a == 11
        && f.get.beams[skeletonBeamCount()].b == skeletonNodeCount(),
        "рука — ствол форка");
    assert(f.get.beams[skeletonBeamCount() + 1].a == 10
        && f.get.beams[skeletonBeamCount() + 1].b == skeletonNodeCount() + 1,
        "зеркальная половина ствола");

    // Пальцы стартуют с той же базы (refBase), а не с конца предыдущей
    // балки; каждый — своя зеркальная пара.
    assert(f.get.beams[skeletonBeamCount() + 2].a == 11
        && f.get.beams[skeletonBeamCount() + 2].b == skeletonNodeCount() + 2
        && f.get.beams[skeletonBeamCount() + 3].a == 10
        && f.get.beams[skeletonBeamCount() + 3].b == skeletonNodeCount() + 3,
        "первый палец и его twin растут из базы сегмента");
    assert(f.get.beams[skeletonBeamCount() + 4].a == 11
        && f.get.beams[skeletonBeamCount() + 4].b == skeletonNodeCount() + 4
        && f.get.beams[skeletonBeamCount() + 5].a == 10
        && f.get.beams[skeletonBeamCount() + 5].b == skeletonNodeCount() + 5,
        "второй палец и его twin — тоже из базы сегмента");

    assert(abs(f.get.nodes[skeletonNodeCount() + 2].pos.x - 0.15f) < 1e-4f
        && abs(f.get.nodes[skeletonNodeCount() + 3].pos.x + 0.15f) < 1e-4f,
        "пальцы зеркальны вокруг оси сегмента");
    assert(abs(f.get.nodes[skeletonNodeCount() + 4].pos.x - 1.15f) < 1e-4f
        && abs(f.get.nodes[skeletonNodeCount() + 5].pos.x + 1.15f) < 1e-4f,
        "второй палец отражается так же, и обе пары симметричны");

    // Пальцы-твины пересекаются в теле на оси сегмента: по правилу
    // «балки не пересекаются» такой каркас невалиден.
    assert(isValidFrame(f.get).isNull, "перекрёст пальцев на оси — невалидный каркас");
}

unittest
{
    // «Глаз циклопа»: в раздвоенном сегменте балка, растущая строго по оси
    // (X == axis), остаётся одиночной и медианной — активатор Nodal не
    // рождает twin из того, что уже на оси. Ось сегмента наследуется от
    // точки старта: спина (refIdx 3) лежит на плоскости X == 0.
    Terminal!Tok[] t;
    // Seed: начало координат (стартовая позиция не расходуется).
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) 3);
    t ~= new Terminal!Tok(Tok.endNew);
    // Балка растёт строго по оси (вперёд×1, вправо/вверх — 0): «глаз».
    t ~= dirCoords(forward, 1.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.2f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.wheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) skeletonNodeCount());

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == skeletonNodeCount() + 1,
        "глаз не рождает twin: нового узла ровно один");
    assert(f.get.beams.length == skeletonBeamCount() + 1,
        "глаз — одиночная балка на оси, без twin");
    assert(f.get.beams[skeletonBeamCount()].a == 3
        && f.get.beams[skeletonBeamCount()].b == skeletonNodeCount(),
        "балка-глаз растёт из спина вперёд");
    assert(cast(Beam) f.get.beams[skeletonBeamCount()] !is null,
        "глаз на оси не задевает ЦМ — обычная балка");
    assert(abs(f.get.nodes[skeletonNodeCount()].pos.x) < 1e-4f
        && abs(f.get.nodes[skeletonNodeCount()].pos.y + 1.205f) < 1e-4f,
        "конец-глаз строго на оси X == 0");
    assert(f.get.anchors.length == 1
        && f.get.anchors[0].node == skeletonNodeCount(),
        "якорь-глаз не дублируется twin'ом");
    assert(!isValidFrame(f.get).isNull, "каркас с глазом остаётся связным");
}

unittest
{
    // Морфоген-градиент: радиус балок масштабируется вдоль порядка
    // построения (0.5 в конце); параметры живут в AST.
    Terminal!Tok[] t;
    // Seed: начало координат.
    t ~= dirCoords(origin, 1.0f);
    t ~= new Terminal!Tok(Tok.taper, 0.5f);
    t ~= new Terminal!Tok(Tok.taperPow, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);

    // Цепочка из двух балок от конца скелета (узел 11) вперёд.
    foreach (_; 0 .. 2)
    {
        t ~= new Terminal!Tok(Tok.refLast);
        t ~= new Terminal!Tok(Tok.endNew);
        t ~= dirCoords(forward, 1.0f);
        t ~= new Terminal!Tok(Tok.radius, 0.06f);
        t ~= new Terminal!Tok(Tok.nodal, 0.0f);
        t ~= new Terminal!Tok(Tok.lefty, 0.0f);
        t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
        t ~= new Terminal!Tok(Tok.turn, 0.0f);
    }
    t ~= new Terminal!Tok(Tok.anchors);

    auto ast = buildAst(t);
    assert(!ast.isNull);
    assert(ast.get.taper == 0.5f && ast.get.taperPow == 1.0f,
        "морфоген читается в AST, а не тонет в потоке");

    auto f = frameFromAst(ast.get);
    assert(!f.isNull);
    assert(f.get.beams.length == skeletonBeamCount() + 2);
    assert(abs(asBeam(f.get.beams[skeletonBeamCount()]).radius - 0.06f) < 1e-5f,
        "первая балка не меняется");
    assert(abs(asBeam(f.get.beams[skeletonBeamCount() + 1]).radius - 0.03f) < 1e-5f,
        "последняя балка сужается в taper раз");
}

unittest
{
    // Вырожденная балка и разорванный каркас отбрасываются.
    Frame f;
    f.nodes = [
        Node(origin),
        Node(right),
    ];
    f.beams = [new Beam(0, 0, 0.04f)];
    f.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(isValidFrame(f).isNull);

    // Простейший валидный каркас: пара узлов с балкой и колёсами.
    Frame g;
    g.nodes = [
        Node(vec3(0.6f, 0.7f, 0.3f)),
        Node(vec3(0.6f, -0.7f, 0.3f)),
    ];
    g.beams = [new Beam(0, 1, 0.05f)];
    g.anchors = [
        Anchor(0, AnchorKind.wheel),
        Anchor(1, AnchorKind.motorWheel),
    ];
    assert(!isValidFrame(g).isNull);
}

unittest
{
    // Длины балок ограничены: минимум 5 см, максимум 3 метра.
    Frame tooShort;
    tooShort.nodes = [Node(origin), Node(vec3(0.04f, 0.0f, 0.0f))];
    tooShort.beams = [new Beam(0, 1, 0.04f)];
    tooShort.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(4.0f < 100.0f * minBeamLength, "балка короче 5 см");
    assert(isValidFrame(tooShort).isNull, "балка короче 5 см — невалидный каркас");

    // Ровно 5 см — на границе допустимого.
    Frame exactMin;
    exactMin.nodes = [Node(origin), Node(vec3(minBeamLength, 0.0f, 0.0f))];
    exactMin.beams = [new Beam(0, 1, 0.04f)];
    exactMin.anchors = [Anchor(0, AnchorKind.wheel), Anchor(1, AnchorKind.wheel)];
    assert(!isValidFrame(exactMin).isNull, "балка ровно 5 см — на границе, валидна");

    Frame tooLong;
    tooLong.nodes = [Node(origin), Node(vec3(3.5f, 0.0f, 0.0f))];
    tooLong.beams = [new Beam(0, 1, 0.04f)];
    tooLong.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(isValidFrame(tooLong).isNull, "балка длиннее 3 м — невалидный каркас");

    Frame exactMax;
    exactMax.nodes = [Node(origin), Node(vec3(maxBeamLength, 0.0f, 0.0f))];
    exactMax.beams = [new Beam(0, 1, 0.04f)];
    exactMax.anchors = [Anchor(0, AnchorKind.wheel), Anchor(1, AnchorKind.wheel)];
    assert(!isValidFrame(exactMax).isNull, "балка ровно 3 м — на границе, валидна");
}

unittest
{
    // Две балки крест-накрест в теле — невалидный каркас.
    Frame x;
    x.nodes = [Node(origin), Node(vec3(1.0f, 0.0f, 0.0f)),
               Node(vec3(0.5f, 0.5f, 0.0f)), Node(vec3(0.5f, -0.5f, 0.0f))];
    x.beams = [new Beam(0, 1, 0.04f), new Beam(2, 3, 0.04f),
               new Beam(0, 2, 0.04f), new Beam(1, 2, 0.04f)];
    x.anchors = [Anchor(0, AnchorKind.wheel), Anchor(3, AnchorKind.wheel)];
    assert(isValidFrame(x).isNull, "перекрёст балок в теле — отбраковка");

    // Общий узел (стык/тройник) пересечением не считается.
    Frame tee;
    tee.nodes = [Node(origin), Node(vec3(1.0f, 0.0f, 0.0f)),
                 Node(vec3(0.5f, 0.5f, 0.0f))];
    tee.beams = [new Beam(0, 1, 0.04f), new Beam(1, 2, 0.04f)];
    tee.anchors = [Anchor(0, AnchorKind.wheel), Anchor(2, AnchorKind.wheel)];
    assert(!isValidFrame(tee).isNull, "стык в общем узле — валиден");
}
