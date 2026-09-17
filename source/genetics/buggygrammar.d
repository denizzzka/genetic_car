module genetics.buggygrammar;

import std.math;
import std.typecons: Nullable;
import dlib.math.vector;
import frame.frame;
import genetics.sge;

enum Tok { refLast, refIdx, endNew, coord, radius, beamKind, anchors, anchorKind }

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
    assert(tok == Tok.refLast || tok == Tok.endNew);

    return new Terminal!Tok(tok);
}

/**
 * Грамматика правой половины багги.
 *
 * Первые три токена `coord` задают позицию seed-узла: x всегда 0 (узел на
 * плоскости симметрии), y и z — случайные. Затем идут балки, затем —
 * якоря (колёса). Узлы отдельно не генерируются: они появляются только как
 * концы балок. Каждая балка: старт — seed/последний созданный узел или
 * существующий по индексу, конец — вновь создаваемый (старт + смещение) или
 * существующий по индексу. Связность правой половины сохраняется, т.к. новые
 * узлы прицепляются к старым, а seed на оси симметрии связывает зеркальные
 * половины полного каркаса (см. `mirrorClosure`, `isConnected`).
 *
 * Якоря генерируются после всех балок: это пара `anchorKind` + `refIdx`,
 * где `refIdx` ссылается на уже созданный узел. Якоря лежат на том же
 * уровне иерархии, что и балки (см. `Frame.anchors`).
 */
Grammar buggyGrammar()
{
    auto beamList_ = nt("beamList", null);

    auto idx = new Sampler!Tok("idx", Tok.refIdx, 64);
    auto destX = new Sampler!Tok("destX", Tok.coord, -1.5f, 1.5f);
    auto destY = new Sampler!Tok("destY", Tok.coord, -1.5f, 1.5f);
    auto destZ = new Sampler!Tok("destZ", Tok.coord, -1.5f, 1.5f);
    auto seedY = new Sampler!Tok("seedY", Tok.coord, -2.0f, 2.0f);
    auto seedZ = new Sampler!Tok("seedZ", Tok.coord, -2.0f, 2.0f);
    auto radius = new Sampler!Tok("radius", Tok.radius, 0.02f, 0.06f);

    auto startRef = nt("startRef", [
        new Production([marker(Tok.refLast)]),
        new Production([idx]),
    ]);

    auto seedPos = nt("seedPos", [
        // X-координата seed-узла всегда 0: узел сажаем на плоскость
        // симметрии (общий для обеих половин), чтобы зеркальные половины
        // полного каркаса были связаны через него. Иначе связность правой
        // половины не гарантирует связности зеркального замыкания.
        new Production([t(Tok.coord, 0.0f), seedY, seedZ]),
    ]);

    auto endRef = nt("endRef", [
        new Production([marker(Tok.endNew), destX, destY, destZ]),
        new Production([idx]),
    ]);

    auto beamKind = nt("beamKind", [
        new Production([t(Tok.beamKind, BeamKind.normal)]),
        new Production([t(Tok.beamKind, BeamKind.cross)]),
        new Production([t(Tok.beamKind, BeamKind.axial)]),
    ]);

    auto beam = nt("beam", [
        new Production([startRef, endRef, radius, beamKind]),
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
        new Production([seedPos, beamList_, anchorMarker, anchorList_]),
    ]);

    auto symbols = [
        start, seedPos, seedY, seedZ, beamList_, beam, startRef, idx,
        endRef, destX, destY, destZ, radius, beamKind,
        anchorMarker, anchorList_, anchor, anchorKind,
    ];
    return new Grammar(start, symbols);
}

/**
 * Разобрать терминалы в правую половину каркаса.
 *
 * Создаётся seed-узел, затем балки по порядку до маркера `Tok.anchors`,
 * затем якоря (каждая пара `anchorKind` + `refIdx` прикрепляет колесо
 * к уже созданному узлу). Токены `refLast` ссылаются на последний созданный
 * узел, `refIdx` — на существующий по номеру, `endNew` (со смещениями)
 * создаёт новый узел на позиции старта + смещение.
 */
bool frameFromTokens(const Terminal!Tok[] tokens, out Frame result)
{
    result = Frame.init;

    // Первые три токена — абсолютная позиция seed-узла.
    if (tokens.length < 3)
        return false;
    if (tokens[0].tok != Tok.coord || tokens[1].tok != Tok.coord || tokens[2].tok != Tok.coord)
        return false;
    auto seed = vec3(tokens[0].f, tokens[1].f, tokens[2].f);
    result.nodes ~= Node(seed);
    size_t last = 0;

    size_t i = 3;
    while (i < tokens.length && tokens[i].tok != Tok.anchors)
    {
        size_t start;
        switch (tokens[i].tok)
        {
            case Tok.refLast:
                start = last;
                ++i;
                break;
            case Tok.refIdx:
                start = tokens[i].i;
                if (start >= result.nodes.length)
                    return false;
                ++i;
                break;
            default:
                return false;
        }

        size_t end;
        switch (tokens[i].tok)
        {
            case Tok.endNew:
                if (i + 3 >= tokens.length)
                    return false;
                if (tokens[i + 1].tok != Tok.coord
                    || tokens[i + 2].tok != Tok.coord
                    || tokens[i + 3].tok != Tok.coord)
                    return false;
                auto dx = tokens[i + 1].f;
                auto dy = tokens[i + 2].f;
                auto dz = tokens[i + 3].f;
                result.nodes ~= Node(result.nodes[start].pos + vec3(dx, dy, dz));
                end = result.nodes.length - 1;
                last = end;
                i += 4;
                break;
            case Tok.refIdx:
                end = tokens[i].i;
                if (end >= result.nodes.length)
                    return false;
                ++i;
                break;
            default:
                return false;
        }

        if (tokens[i].tok != Tok.radius)
            return false;
        auto radius = tokens[i].f;
        ++i;

        if (tokens[i].tok != Tok.beamKind)
            return false;
        auto beamKind = cast(BeamKind) tokens[i].i;
        ++i;

        result.beams ~= Beam(start, end, radius, beamKind);
    }

    if (i >= tokens.length)
        return true;

    // Маркер `Tok.anchors` — переход к якорям.
    ++i;
    while (i < tokens.length)
    {
        if (tokens[i].tok != Tok.anchorKind)
            return false;
        auto kind = cast(AnchorKind) tokens[i].i;
        ++i;

        if (i >= tokens.length || tokens[i].tok != Tok.refIdx)
            return false;
        // Индекс узла, как и кодоны SGE, заворачивается по числу узлов:
        // случайный индекс за пределами каркаса прижимается к существующему узлу,
        // а не роняет весь кадр.
        auto n = cast(size_t) tokens[i].i % result.nodes.length;
        ++i;

        result.anchors ~= Anchor(n, kind);
    }
    return true;
}

/// Минимальная проверка каркаса: узлы в границах, без вырожденных балок,
/// весь (в т.ч. зеркальный) каркас — один связный граф, якоря — на валидных узлах.
bool isValidFrame(const Frame f)
{
    if (f.nodes.length == 0 || f.beams.length == 0)
        return false;
    // Правая половина каркаса: узлы не могут уходить на левую сторону
    // (x < 0) — иначе зеркальное замыкание сломает инвариант половины.
    foreach (n; f.nodes)
        if (n.pos.x < -planeEpsilon)
            return false;
    foreach (b; f.beams)
    {
        if (b.a >= f.nodes.length || b.b >= f.nodes.length)
            return false;
        const pa = f.nodes[b.a].pos;
        const pb = f.nodes[b.b].pos;
        if (b.a == b.b || distance(pa, pb) < 1e-4f)
            return false;

        //TODO: Возможно надо переключать тип балкиесли она перестала удовлетворять критериям типа
        const aOnPlane = isOnPlane(pa);
        const bOnPlane = isOnPlane(pb);
        final switch (b.kind)
        {
            case BeamKind.normal:
                break;
            case BeamKind.cross:
                // Правый узел a (вне плоскости) к осевому b (x == 0),
                // на тех же y, z — ровно так, как ожидает mirrorClosure.
                if (aOnPlane || !bOnPlane)
                    return false;
                if (!isClose(pa.y, pb.y) || !isClose(pa.z, pb.z))
                    return false;
                break;
            case BeamKind.axial:
                // Оба конца на оси симметрии.
                if (!aOnPlane || !bOnPlane)
                    return false;
                break;
        }
    }

    // Якоря (колёса): валидный узел, не на оси симметрии.
    // Дубли на одном узле допускаются — это вырожденный случай, который
    // отсеется на этапе физики/фитнеса, а не на этапе синтаксиса.
    foreach (a; f.anchors)
    {
        if (a.node >= f.nodes.length)
            return false;
        if (isOnPlane(f.nodes[a.node].pos))
            return false;
    }

    // Связность проверяется на полном (зеркальном) каркасе. Связность
    // правой половины сама по себе не гарантирует, что mirrorClosure не
    // распадётся на отдельные компоненты: cross-балка в полном каркасе
    // соединяет правый узел со своим зеркалом, а осевой узел такой балки
    // вообще не получает рёбер, поэтому «висящие» cross-трубы и разорванные
    // пополам половины возможны даже при связной половине.
    if (!isConnected(mirrorClosure(f)))
        return false;
    return true;
}

/// Расшифровать геном из грамматики багги в кадр (фенотип).
/// Значение-результат сам говорит об успехе: `Nullable!Frame.isNull`
/// означает, что декодирование, разбор или валидация не прошли.
Nullable!Frame develop(const Grammar gr, const Genotype g)
{
    auto tokens = decode!Tok(gr, g);
    if (tokens is null)
        return Nullable!Frame.init;
    Frame result;
    if (!frameFromTokens(tokens, result))
        return Nullable!Frame.init;
    if (!isValidFrame(result))
        return Nullable!Frame.init;
    return Nullable!Frame(result);
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

        Frame f;
        if (!frameFromTokens(tokens, f))
            continue;

        if (isValidFrame(f))
        {
            ++valid;
            assert(f.nodes.length > 0);
            assert(f.beams.length > 0);
            assert(f.anchors.length > 0);
        }
    }

    // Декодирование сходится почти всегда; значительная доля геномов даёт
    // валидный каркас (остальные отбрасываются самокоррекцией — ссылки на
    // узлы, которых ещё нет, вырожденные балки, cross/axial вне оси).
    // С непрерывными самплерами и проверкой связности зеркального каркаса
    // доля валидных ~0.8% (см. scripts/probe.d).
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
    // Половина связана, но зеркальное замыкание распадается: такой каркас
    // обязан быть отброшен проверкой связности полного каркаса.

    // Висящая cross-труба: у off-plane узла только cross-балка к осевому.
    {
        Frame f;
        f.nodes = [
            Node(vec3(0.3f, 0.0f, 0.0f)),
            Node(vec3(0.0f, 0.0f, 0.0f)),
        ];
        f.beams = [Beam(0, 1, 0.04f, BeamKind.cross)];
        assert(!isValidFrame(f));
    }

    // Разорванные половины: обычная балка между двумя off-plane узлами,
    // связующих cross/осевого узла нет.
    {
        Frame f;
        f.nodes = [
            Node(vec3(0.3f, 0.0f, 0.0f)),
            Node(vec3(0.3f, 1.0f, 0.0f)),
        ];
        f.beams = [Beam(0, 1, 0.04f, BeamKind.normal)];
        assert(!isValidFrame(f));
    }

    // Осевой узел-мост: связно и до, и после зеркального замыкания.
    {
        Frame f;
        f.nodes = [
            Node(vec3(0.3f, 0.0f, 0.0f)),
            Node(vec3(0.0f, 0.0f, 0.0f)),
        ];
        f.beams = [Beam(0, 1, 0.04f, BeamKind.normal)];
        assert(isValidFrame(f));
    }
}
