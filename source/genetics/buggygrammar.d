module genetics.buggygrammar;

import std.math;
import std.typecons: Nullable;
import dlib.math.vector;
import frame.frame;
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
    // Сила мотор-колёс, Н·м: от почти стоячей машины до агрессивной,
    // способной вилли; эволюция ищет окно между «не едет» и «опрокидывается».
    auto motorPower = new Sampler!Tok("motorPower", Tok.motorPower, 0.0f, 200.0f);

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

    auto anchor = nt("anchor", [
        new Production([anchorKind, idx]),
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
        anchorMarker, anchorList_, anchor, anchorKind,
    ];
    return new Grammar(start, symbols);
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

    size_t last = addNode(ast.seed, false, 0.0f);

    float heading = ast.heading;
    auto forward = (float dx, float dy, float dz) {
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
                        + forward(b.end.delta.x, b.end.delta.y, b.end.delta.z);
                    end = addNode(target, seg.fork, axis);
                    last = end;
                    break;
                }
                case EndRefKind.nearNode:
                {
                    auto target = result.nodes[start].pos
                        + forward(b.end.delta.x, b.end.delta.y, b.end.delta.z);
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

            // Пары: балка и её twin, twin-балку растаскивает активатор
            // Nodal, ингибитор Lefty гасит его (self-limiting асимметрия).
            result.beams ~= Beam(start, end, radius, b.kind);
            if (seg.fork && !(forkOf[start] == start && forkOf[end] == end))
                result.beams ~= Beam(forkOf[start], forkOf[end],
                    radius * (1.0f + forkAsymmetry(b.nodal, b.lefty)), b.kind);

            heading += b.turn;
        }
    }

    foreach (a; ast.anchors)
    {
        auto n = cast(size_t) a.idx % result.nodes.length;
        result.anchors ~= Anchor(n, a.kind);
        if (forkOf[n] != n)
            result.anchors ~= Anchor(forkOf[n], a.kind);
    }
    result.motorPower = ast.motorPower;
    return Nullable!Frame(result);
}

enum float minBeamLength = 0.05f;
enum float maxBeamLength = 3.0f;

/**
 * Структурная валидность каркаса: узлы в границах, без вырожденных балок,
 * все балки в допустимом диапазоне длин, якоря — на валидных узлах,
 * каркас связен.
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
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    t ~= new Terminal!Tok(Tok.coord, -0.95f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 2,
        "endNear должен слиться с узлом 0, а не создавать новый");
    assert(f.get.beams.length == 2);
    assert(f.get.beams[1].a == 1 && f.get.beams[1].b == 0,
        "вторая балка замыкает петлю на существующий узел");
    assert(!isValidFrame(f.get).isNull, "замкнутая петля остаётся связной");
}

unittest
{
    // endNear далеко от структуры ведёт себя как endNew — новый узел.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 2);
    assert(f.get.beams.length == 1);
}

unittest
{
    // Turtle: дельты интерпретируются в системе заголовка, а turn накапливает
    // направление. Заголовок π/2 поворачивает дельту (1,0,0) в мировые (0,1,0).
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 1.5707963f);
    t ~= new Terminal!Tok(Tok.segStart);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 3);
    const n1 = f.get.nodes[1].pos;
    const n2 = f.get.nodes[2].pos;
    assert(n1.x < 1e-4f && n1.y > 0.999f,
        "заголовок π/2 должен развернуть дельту из оси X в ось Y");
    assert(n2.x < -0.999f && n2.y > 0.999f,
        "дельта (0,1,0) при заголовке π/2 уходит влево (-X)");
}

unittest
{
    // Раздвоение (fork): каждая балка раздвоенного сегмента рождает twin вокруг
    // оси сегмента — X его стартового узла. Старт на узле (0,0,0) даёт ось X==0,
    // поэтому пары геометрически симметричны вокруг нуля.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.1f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.1f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.wheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) 1);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 5,
        "каждый узел раздвоенного сегмента (кроме оси) рождается парой с twin");
    assert(abs(f.get.nodes[3].pos.x - 1.0f) < 1e-4f
        && abs(f.get.nodes[3].pos.y - 1.0f) < 1e-4f
        && abs(f.get.nodes[4].pos.x + 1.0f) < 1e-4f
        && abs(f.get.nodes[4].pos.y - 1.0f) < 1e-4f,
        "twin отражается геометрически вокруг оси: (1,1) -> (-1,1)");

    assert(f.get.beams.length == 4);
    assert(f.get.beams[0].a == 0 && f.get.beams[0].b == 1);
    assert(f.get.beams[1].a == 0 && f.get.beams[1].b == 2,
        "вторая балка — twin первой через ось сегмента");
    assert(f.get.beams[2].a == 1 && f.get.beams[2].b == 3);
    assert(f.get.beams[3].a == 2 && f.get.beams[3].b == 4);

    // Активатор Nodal=0.1 без ингибитора даёт сдвиг s=0.1,
    // radius twin-балки: 0.04 * (1 + 0.1) = 0.044.
    assert(abs(f.get.beams[1].radius - 0.044f) < 1e-4f);
    assert(abs(f.get.beams[3].radius - 0.044f) < 1e-4f);

    assert(f.get.anchors.length == 2,
        "якорь дублируется на twin-узел");
    assert(f.get.anchors[0].node == 1 && f.get.anchors[1].node == 2);

    assert(!isValidFrame(f.get).isNull,
        "раздвоенный каркас с узлом на оси остаётся связным");
}

unittest
{
    // Раздвоение поверх turtle: twin-половина поворачивается вместе
    // с заголовком. Заголовок π/4 разворачивает дельту (1,0,0) в (c,c),
    // twin — в (-c,c) вокруг оси сегмента (X узла старта).
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.78539815f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 3);
    const n1 = f.get.nodes[1].pos;
    const n2 = f.get.nodes[2].pos;
    assert(n1.x > 0.7071f && n1.x < 0.7072f && n1.y > 0.7071f && n1.y < 0.7072f,
        "заголовок π/4 поворачивает дельту (1,0,0) в (c,c)");
    assert(n2.x < -0.7071f && n2.x > -0.7072f && abs(n2.y - n1.y) < 1e-4f,
        "twin отражает узел (c,c) в (-c,c) вокруг оси сегмента");
}

unittest
{
    // Ось раздвоения — локальная (X узла старта сегмента), а не мировая X == 0.
    // Старт вне нуля не прижимается и не рвёт каркас: обе ветви держатся
    // на узле старта, он в своей же оси и сам остаётся одиночным.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.3f);
    t ~= new Terminal!Tok(Tok.coord, 0.2f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 3,
        "внеосевой старт: ветвь (1.3) + twin (2*0.3-1.3) вокруг оси 0.3");
    assert(abs(f.get.nodes[1].pos.x - 1.3f) < 1e-4f,
        "ветвь растёт от реального старта, старт не прижимается к нулю");
    assert(abs(f.get.nodes[2].pos.x + 0.7f) < 1e-4f,
        "twin отражается вокруг оси 0.3, а не мировой X == 0");
    assert(!isValidFrame(f.get).isNull,
        "обе ветви связаны на узле старта — каркас связен");
}

unittest
{
    // Сегмент без раздвоения — медианная одиночная структура без twin:
    // ни один узел не дублируется.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);

    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.motorWheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) 1);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 2);
    assert(f.get.beams.length == 1);
    assert(f.get.anchors.length == 1,
        "без раздвоения якорь не дублируется");
}

unittest
{
    // refBase — «зачаток»: балка стартует с базового узла сегмента, а не
    // с последнего. Из одной точки ветвления (база «запястья») выпускаются
    // несколько отростков-«пальцев», каждый в своей зеркальной паре.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refBase);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 0.5f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refBase);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.5f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.0f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = toFrame(t);
    assert(!f.isNull);
    // 1 база + пара рук + два раза по паре пальцев = 7 узлов.
    assert(f.get.nodes.length == 7);
    assert(f.get.beams.length == 6, "три балки раздвоенного сегмента дают три пары");

    // Рука: от базы к (1,0,0) и её twin.
    assert(f.get.beams[0].a == 0 && f.get.beams[0].b == 1);
    assert(f.get.beams[1].a == 0 && f.get.beams[1].b == 2);

    // Пальцы стартуют с того же «запястья» (узел 0) — refBase, а не с конца
    // предыдущей балки (node 1). Каждый палец — своя зеркальная пара.
    assert(f.get.beams[2].a == 0 && f.get.beams[2].b == 3
        && f.get.beams[3].a == 0 && f.get.beams[3].b == 4,
        "первый палец и его twin растут из базы сегмента");
    assert(f.get.beams[4].a == 0 && f.get.beams[4].b == 5
        && f.get.beams[5].a == 0 && f.get.beams[5].b == 6,
        "второй палец и его twin — тоже из базы сегмента");

    assert(abs(f.get.nodes[3].pos.x - 0.5f) < 1e-4f
        && abs(f.get.nodes[4].pos.x + 0.5f) < 1e-4f,
        "пальцы зеркальны вокруг оси сегмента");
    assert(abs(f.get.nodes[5].pos.x - 1.5f) < 1e-4f
        && abs(f.get.nodes[6].pos.x + 1.5f) < 1e-4f,
        "второй палец отражается так же, и обе пары симметричны");

    assert(!isValidFrame(f.get).isNull, "каркас с пальцами остаётся связным");
}

unittest
{
    // «Глаз циклопа»: даже в раздвоенном сегменте балка, растущая строго по
    // оси (X == axis), остаётся одиночной и медианной — активатор Nodal не
    // рождает twin из того, что на оси. Ось наследуется верно и при этом.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.7f);
    t ~= new Terminal!Tok(Tok.coord, 0.2f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.2f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.wheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) 1);

    auto f = toFrame(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 2, "балка на оси не рождает twin");
    assert(f.get.beams.length == 1, "сильный активатор не дублирует «глаз»");
    assert(abs(f.get.nodes[1].pos.x - 0.7f) < 1e-4f,
        "«глаз циклопа» остаётся на медианной оси сегмента");
    assert(f.get.anchors.length == 1,
        "одиночный медианный узел не дублируется в якорях");
}

unittest
{
    // Морфоген-градиент: радиус балок масштабируется вдоль порядка
    // построения (0.5 в конце); параметры живут в AST.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.taper, 0.5f);
    t ~= new Terminal!Tok(Tok.taperPow, 1.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);

    foreach (_; 0 .. 2)
    {
        t ~= new Terminal!Tok(Tok.refLast);
        t ~= new Terminal!Tok(Tok.endNew);
        t ~= new Terminal!Tok(Tok.coord, 1.0f);
        t ~= new Terminal!Tok(Tok.coord, 0.0f);
        t ~= new Terminal!Tok(Tok.coord, 0.0f);
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
    assert(f.get.beams.length == 2);
    assert(abs(f.get.beams[0].radius - 0.06f) < 1e-5f,
        "первая балка не меняется");
    assert(abs(f.get.beams[1].radius - 0.03f) < 1e-5f,
        "последняя балка сужается в taper раз");
}

unittest
{
    // Вырожденная балка и разорванный каркас отбрасываются.
    Frame f;
    f.nodes = [
        Node(vec3(0.0f, 0.0f, 0.0f)),
        Node(vec3(0.0f, 1.0f, 0.0f)),
    ];
    f.beams = [Beam(0, 0, 0.04f)];
    f.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(isValidFrame(f).isNull);

    // Простейший валидный каркас: пара узлов с балкой и колёсами.
    Frame g;
    g.nodes = [
        Node(vec3(0.6f, 0.7f, 0.3f)),
        Node(vec3(0.6f, -0.7f, 0.3f)),
    ];
    g.beams = [Beam(0, 1, 0.05f)];
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
    tooShort.nodes = [Node(vec3(0.0f, 0.0f, 0.0f)), Node(vec3(0.04f, 0.0f, 0.0f))];
    tooShort.beams = [Beam(0, 1, 0.04f)];
    tooShort.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(4.0f < 100.0f * minBeamLength, "балка короче 5 см");
    assert(isValidFrame(tooShort).isNull, "балка короче 5 см — невалидный каркас");

    // Ровно 5 см — на границе допустимого.
    Frame exactMin;
    exactMin.nodes = [Node(vec3(0.0f, 0.0f, 0.0f)), Node(vec3(minBeamLength, 0.0f, 0.0f))];
    exactMin.beams = [Beam(0, 1, 0.04f)];
    exactMin.anchors = [Anchor(0, AnchorKind.wheel), Anchor(1, AnchorKind.wheel)];
    assert(!isValidFrame(exactMin).isNull, "балка ровно 5 см — на границе, валидна");

    Frame tooLong;
    tooLong.nodes = [Node(vec3(0.0f, 0.0f, 0.0f)), Node(vec3(3.5f, 0.0f, 0.0f))];
    tooLong.beams = [Beam(0, 1, 0.04f)];
    tooLong.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(isValidFrame(tooLong).isNull, "балка длиннее 3 м — невалидный каркас");

    Frame exactMax;
    exactMax.nodes = [Node(vec3(0.0f, 0.0f, 0.0f)), Node(vec3(maxBeamLength, 0.0f, 0.0f))];
    exactMax.beams = [Beam(0, 1, 0.04f)];
    exactMax.anchors = [Anchor(0, AnchorKind.wheel), Anchor(1, AnchorKind.wheel)];
    assert(!isValidFrame(exactMax).isNull, "балка ровно 3 м — на границе, валидна");
}
