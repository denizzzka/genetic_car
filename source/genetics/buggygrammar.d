module genetics.buggygrammar;

import std.math;
import std.random: Random, uniform;
import std.typecons: Nullable;
import dlib.math.vector;
import frame.frame;
import genetics.sge;
import genetics.buggyast;

/**
 * Терминалы грамматики багги — «команды развития», которые `decode` выводит
 * из генома. `buildAst` собирает из них дерево, `frameFromAst` выполняет
 * в порядке следования, строя каркас от первого узла. Часть токенов несёт
 * значение (float/int в `payload`), часть — пустые маркеры.
 */
enum Tok
{
    /// Маркер начала сегмента: ограничивает группу балок, строящихся как
    /// одно модульное целое. Каждый сегмент сам решает, раздваиваться ли.
    segStart,

    /// Маркер раздвоения сегмента: если стоит после `segStart`, каждая балка
    /// сегмента рождает пару (twin) относительно оси сегмента — X стартового
    /// узла сегмента (не мировой X == 0). Ось дают локальные пары конечностей;
    /// пустой маркер — медианная одиночная структура («глаз по центру»).
    fork,

    /// Коэффициент асимметрии пары (float, ±0.1): радиус twin-балки
    /// масштабируется в `1 + forkDelta` раз. Нулевое значение — пара
    /// зеркально-точная, отбор может эволюционировать направленную
    /// асимметрию между сторонами (как клешни краба).
    forkDelta,

    /// Начало балки: ссылка на последний созданный узел (маркер без значения).
    refLast,

    /// Ссылка на существующий узел по номеру (int): начало/конец балки
    /// или индекс узла якоря. В якорях заворачивается по числу узлов.
    refIdx,

    /// Маркер конца балки: создать новый узел на позиции `старта + смещение`
    /// (после маркера идут три токена `coord`).
    endNew,

    /// Маркер конца балки: как `endNew`, но растущий конец сливается
    /// с ближайшим существующим узлом (кроме старта) в пределах
    /// `mergeRadius`; иначе — как `endNew`.
    endNear,

    /// Координата (float): абсолютная позиция первого узла (seed)
    /// или дельта `destX/destY/destZ` относительно старта балки.
    coord,

    /// Радиус трубы балки (float, базовый семпл 0.02..0.06),
    /// до интерпретации масштабируется морфоген-градиентом.
    radius,

    /// Тип балки (int-код `BeamKind`).
    beamKind,

    /// Маркер: конец списка балок, начало списка якорей (колёс).
    anchors,

    /// Тип якоря-колеса (int-код `AnchorKind`).
    anchorKind,

    /// Морфоген: во сколько раз сужается толщина к последней балке
    /// (float, 0.4..1.0). Токен изымается из потока до интерпретации.
    taper,

    /// Морфоген: показатель степени кривой градиента толщины
    /// (float, 0.5..4.0). Токен изымается из потока до интерпретации.
    taperPow,

    /// Turtle: начальный заголовок построения (float, радианы). Дельты
    /// `endNew`/`endNear` интерпретируются в системе заголовка: `coord` X —
    /// вперёд по заголовку, `coord` Y — вправо от него, Z — вверх.
    heading,

    /// Turtle: приращение заголовка после балки (float, радианы).
    /// Накапливается в общее направление построения.
    turn,
}

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
    assert(tok == Tok.refLast || tok == Tok.endNew || tok == Tok.endNear
        || tok == Tok.segStart || tok == Tok.fork || tok == Tok.anchors);

    return new Terminal!Tok(tok);
}

/**
 * Грамматика багги целиком.
 *
 * Каркас — последовательность сегментов, модульных групп балок. Каждый
 * сегмент начинается маркером `segStart`, за ним идёт необязательный
 * `Tok.fork` (раздвоение) и `forkDelta` — коэффициент асимметрии пары.
 * Узлы отдельно не генерируются: они появляются только как концы балок.
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
    // оси (X стартового узла сегмента).
    auto segMode = nt("segMode", [
        new Production([]),
        new Production([marker(Tok.fork)]),
    ]);

    auto forkDelta = new Sampler!Tok("forkDelta", Tok.forkDelta, -0.1f, 0.1f);

    auto segment = nt("segment", [
        new Production([marker(Tok.segStart), segMode, forkDelta, beamList_]),
    ]);

    segmentList_.productions = [
        new Production([segment, segmentList_]),
        new Production([segment]),
    ];

    auto idx = new Sampler!Tok("idx", Tok.refIdx, 64);
    auto destX = new Sampler!Tok("destX", Tok.coord, -1.5f, 1.5f);
    auto destY = new Sampler!Tok("destY", Tok.coord, -1.5f, 1.5f);
    auto destZ = new Sampler!Tok("destZ", Tok.coord, -1.5f, 1.5f);
    auto startX = new Sampler!Tok("startX", Tok.coord, -2.0f, 2.0f);
    auto startY = new Sampler!Tok("startY", Tok.coord, -2.0f, 2.0f);
    auto startZ = new Sampler!Tok("startZ", Tok.coord, -2.0f, 2.0f);
    auto radius = new Sampler!Tok("radius", Tok.radius, 0.02f, 0.06f);
    auto taper = new Sampler!Tok("taper", Tok.taper, 0.4f, 1.0f);
    auto taperPow = new Sampler!Tok("taperPow", Tok.taperPow, 0.5f, 4.0f);
    auto heading = new Sampler!Tok("heading", Tok.heading, -3.1416f, 3.1416f);
    auto turn = new Sampler!Tok("turn", Tok.turn, -1.5708f, 1.5708f);

    auto startRef = nt("startRef", [
        new Production([marker(Tok.refLast)]),
        new Production([idx]),
    ]);

    auto startPos = nt("startPos", [
        new Production([startX, startY, startZ, taper, taperPow, heading]),
    ]);

    auto endRef = nt("endRef", [
        new Production([marker(Tok.endNew), destX, destY, destZ]),
        new Production([idx]),
        new Production([marker(Tok.endNear), destX, destY, destZ]),
    ]);

    auto beamKind = nt("beamKind", [
        new Production([t(Tok.beamKind, BeamKind.normal)]),
    ]);

    auto beam = nt("beam", [
        new Production([startRef, endRef, radius, beamKind, turn]),
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
        start, startPos, startX, startY, startZ, taper, taperPow, heading, turn,
        segmentList_, segment, segMode, forkDelta,
        beamList_, beam, startRef,
        idx, endRef, destX, destY, destZ, radius, beamKind,
        anchorMarker, anchorList_, anchor, anchorKind,
    ];
    return new Grammar(start, symbols);
}

/**
 * Построить AST развития из потока терминалов.
 *
 * Дерево хранит все параметры грамматики без геометрии: startPos (seed,
 * heading, морфоген taper/taperPow), сегменты (fork, forkDelta, балки),
 * балки (старт/конец как типизированные рефы, радиус, turn) и якоря.
 * Геометрия здесь не строится — это этап разбора, а не интерпретации.
 */
Nullable!Ast buildAst(const Terminal!Tok[] tokens)
{
    Ast ast;
    size_t i = 0;

    if (i + 2 >= tokens.length)
        return Nullable!Ast.init;
    if (tokens[i].tok != Tok.coord || tokens[i + 1].tok != Tok.coord || tokens[i + 2].tok != Tok.coord)
        return Nullable!Ast.init;
    ast.seed = vec3(tokens[i].f, tokens[i + 1].f, tokens[i + 2].f);
    i += 3;

    // Морфоген-градиент толщины — параметры каркаса, читаются из startPos.
    if (i < tokens.length && tokens[i].tok == Tok.taper)
    {
        ast.taper = tokens[i].f;
        ++i;
    }
    if (i < tokens.length && tokens[i].tok == Tok.taperPow)
    {
        ast.taperPow = tokens[i].f;
        ++i;
    }

    if (tokens.length <= i || tokens[i].tok != Tok.heading)
        return Nullable!Ast.init;
    ast.heading = tokens[i].f;
    ++i;

    while (i < tokens.length && tokens[i].tok != Tok.anchors)
    {
        if (tokens[i].tok != Tok.segStart)
            return Nullable!Ast.init;
        ++i;

        SegmentAst seg;
        if (i < tokens.length && tokens[i].tok == Tok.fork)
        {
            seg.fork = true;
            ++i;
        }

        if (i >= tokens.length || tokens[i].tok != Tok.forkDelta)
            return Nullable!Ast.init;
        seg.forkDelta = tokens[i].f;
        ++i;

        while (i < tokens.length && tokens[i].tok != Tok.segStart
            && tokens[i].tok != Tok.anchors)
        {
            BeamAst b;
            switch (tokens[i].tok)
            {
                case Tok.refLast:
                    b.start.kind = StartRefKind.last;
                    ++i;
                    break;
                case Tok.refIdx:
                    b.start.kind = StartRefKind.idx;
                    b.start.idx = tokens[i].i;
                    ++i;
                    break;
                default:
                    return Nullable!Ast.init;
            }

            switch (tokens[i].tok)
            {
                case Tok.endNew:
                    if (i + 3 >= tokens.length)
                        return Nullable!Ast.init;
                    if (tokens[i + 1].tok != Tok.coord
                        || tokens[i + 2].tok != Tok.coord
                        || tokens[i + 3].tok != Tok.coord)
                        return Nullable!Ast.init;
                    b.end.kind = EndRefKind.newNode;
                    b.end.delta = vec3(tokens[i + 1].f, tokens[i + 2].f, tokens[i + 3].f);
                    i += 4;
                    break;
                case Tok.endNear:
                    if (i + 3 >= tokens.length)
                        return Nullable!Ast.init;
                    if (tokens[i + 1].tok != Tok.coord
                        || tokens[i + 2].tok != Tok.coord
                        || tokens[i + 3].tok != Tok.coord)
                        return Nullable!Ast.init;
                    b.end.kind = EndRefKind.nearNode;
                    b.end.delta = vec3(tokens[i + 1].f, tokens[i + 2].f, tokens[i + 3].f);
                    i += 4;
                    break;
                case Tok.refIdx:
                    b.end.kind = EndRefKind.idx;
                    b.end.idx = tokens[i].i;
                    ++i;
                    break;
                default:
                    return Nullable!Ast.init;
            }

            if (tokens[i].tok != Tok.radius)
                return Nullable!Ast.init;
            b.radius = tokens[i].f;
            ++i;

            if (tokens[i].tok != Tok.beamKind)
                return Nullable!Ast.init;
            b.kind = cast(BeamKind) tokens[i].i;
            ++i;

            if (tokens[i].tok != Tok.turn)
                return Nullable!Ast.init;
            b.turn = tokens[i].f;
            ++i;

            seg.beams ~= b;
        }
        ast.segments ~= seg;
    }

    if (i >= tokens.length)
        return Nullable!Ast(ast);

    // Якоря: пара `anchorKind` + `refIdx` (индекс узла, заворачивается по
    // числу узлов в интерпретаторе — синтаксис тут ничего не решает).
    ++i;
    while (i < tokens.length)
    {
        if (tokens[i].tok != Tok.anchorKind)
            return Nullable!Ast.init;
        auto kind = cast(AnchorKind) tokens[i].i;
        ++i;

        if (i >= tokens.length || tokens[i].tok != Tok.refIdx)
            return Nullable!Ast.init;
        AnchorAst a;
        a.kind = kind;
        a.idx = tokens[i].i;
        ++i;

        ast.anchors ~= a;
    }
    return Nullable!Ast(ast);
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
                case StartRefKind.idx:
                    start = b.start.idx;
                    if (start >= result.nodes.length)
                        return Nullable!Frame.init;
                    break;
            }

            if (seg.fork && !haveAxis)
            {
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

            // Пары: балка и её twin, radius twin-балки растаскивается forkDelta.
            result.beams ~= Beam(start, end, radius, b.kind);
            if (seg.fork && !(forkOf[start] == start && forkOf[end] == end))
                result.beams ~= Beam(forkOf[start], forkOf[end],
                    radius * (1.0f + seg.forkDelta), b.kind);

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
    return Nullable!Frame(result);
}

/**
 * Структурная валидность каркаса: узлы в границах, без вырожденных балок,
 * якоря — на валидных узлах, каркас связен.
 */
Nullable!Frame isValidFrame(Frame f)
{
    if (f.nodes.length == 0 || f.beams.length == 0)
        return Nullable!Frame.init;
    foreach (b; f.beams)
    {
        if (b.a >= f.nodes.length || b.b >= f.nodes.length)
            return Nullable!Frame.init;
        const pa = f.nodes[b.a].pos;
        const pb = f.nodes[b.b].pos;
        if (b.a == b.b || distance(pa, pb) < 1e-4f)
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

/**
 * Один шаг мутации генома с проверкой развития.
 *
 * Обычно — точечные правки (`mutate`). С вероятностью 25% — индельная
 * мутация (`mutateIndel`), единственная, что меняет длины генов и, значит,
 * число балок и колёс. Индел принимается, только если он действительно
 * изменил структуру: иначе редкая структурная правка тонет среди
 * геометрических, и каркас никогда не растёт.
 *
 * Возврат — сам результат: `Nullable!Genotype.isNull` означает, что подходящий
 * мутант не нашёлся.
 */
Nullable!Genotype mutateStep(const Grammar gr, const Genotype genome, ref Random rnd)
{
    const current = develop(gr, genome);
    if (current.isNull)
        return Nullable!Genotype.init;

    const structural = uniform(0.0f, 1.0f, rnd) < 0.25f;
    foreach (_; 0 .. 100)
    {
        auto candidate = genome.dup;
        if (structural)
            mutateIndel(candidate, 1, rnd);
        else
            mutate(candidate, 1 + uniform(0u, 3u, rnd), rnd);

        auto may = develop(gr, candidate);
        if (may.isNull)
            continue;
        if (structural
            && may.get.frame.beams.length == current.get.frame.beams.length
            && may.get.frame.anchors.length == current.get.frame.anchors.length)
            continue;

        return Nullable!Genotype(candidate);
    }
    return Nullable!Genotype.init;
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
    t ~= new Terminal!Tok(Tok.forkDelta, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    t ~= new Terminal!Tok(Tok.coord, -0.95f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
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
    t ~= new Terminal!Tok(Tok.forkDelta, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
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
    t ~= new Terminal!Tok(Tok.forkDelta, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
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
    // AST: структура грамматики видна без геометрии — сегмент, рефы, якоря.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.5f);
    t ~= new Terminal!Tok(Tok.coord, -0.25f);
    t ~= new Terminal!Tok(Tok.coord, 0.1f);
    t ~= new Terminal!Tok(Tok.taper, 0.6f);
    t ~= new Terminal!Tok(Tok.taperPow, 2.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.3f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.forkDelta, 0.05f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.05f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.7f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.motorWheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) 3);

    auto ast = buildAst(t);
    assert(!ast.isNull);
    assert(abs(ast.get.seed.x - 0.5f) < 1e-6f && abs(ast.get.seed.y + 0.25f) < 1e-6f);
    assert(ast.get.taper == 0.6f && ast.get.taperPow == 2.0f);
    assert(abs(ast.get.heading - 0.3f) < 1e-6f);

    assert(ast.get.segments.length == 1);
    const seg = ast.get.segments[0];
    assert(seg.fork && abs(seg.forkDelta - 0.05f) < 1e-6f);
    assert(seg.beams.length == 1);
    assert(seg.beams[0].start.kind == StartRefKind.last);
    assert(seg.beams[0].end.kind == EndRefKind.nearNode);
    assert(abs(seg.beams[0].end.delta.x - 1.0f) < 1e-6f);
    assert(abs(seg.beams[0].turn - 0.7f) < 1e-6f);

    assert(ast.get.anchors.length == 1);
    assert(ast.get.anchors[0].kind == AnchorKind.motorWheel);
    assert(ast.get.anchors[0].idx == 3);
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
    t ~= new Terminal!Tok(Tok.forkDelta, 0.1f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
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

    // forkDelta растаскивает радиус twin-балки: 0.04 * (1 + 0.1) = 0.044.
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
    t ~= new Terminal!Tok(Tok.forkDelta, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
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
    t ~= new Terminal!Tok(Tok.forkDelta, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
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
    t ~= new Terminal!Tok(Tok.forkDelta, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
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
    t ~= new Terminal!Tok(Tok.forkDelta, 0.0f);
    foreach (_; 0 .. 2)
    {
        t ~= new Terminal!Tok(Tok.refLast);
        t ~= new Terminal!Tok(Tok.endNew);
        t ~= new Terminal!Tok(Tok.coord, 1.0f);
        t ~= new Terminal!Tok(Tok.coord, 0.0f);
        t ~= new Terminal!Tok(Tok.coord, 0.0f);
        t ~= new Terminal!Tok(Tok.radius, 0.06f);
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