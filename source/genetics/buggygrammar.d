module genetics.buggygrammar;

import std.math;
import std.typecons: Nullable;
import dlib.math.vector;
import frame.frame;
import genetics.sge;

enum Tok { refLast, refIdx, endNew, coord, radius, beamKind }

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
 * Первые три токена `coord` задают абсолютную позицию seed-узла
 * (любую, не фиксированную), затем идут балки. Узлы отдельно не
 * генерируются: они появляются только как концы балок. Каждая балка:
 * старт — seed/последний созданный узел или существующий по индексу,
 * конец — вновь создаваемый (старт + смещение) или существующий по индексу.
 * Связность сохраняется, т.к. новые узлы прицепляются к старым.
 */
Grammar buggyGrammar()
{
    auto beamList_ = nt("beamList", null);

    auto idx = new Sampler!Tok("idx", Tok.refIdx, 64);
    auto destX = new Sampler!Tok("destX", Tok.coord, -1.5f, 1.5f);
    auto destY = new Sampler!Tok("destY", Tok.coord, -1.5f, 1.5f);
    auto destZ = new Sampler!Tok("destZ", Tok.coord, -1.5f, 1.5f);
    auto seedX = new Sampler!Tok("seedX", Tok.coord, -2.0f, 2.0f);
    auto seedY = new Sampler!Tok("seedY", Tok.coord, -2.0f, 2.0f);
    auto seedZ = new Sampler!Tok("seedZ", Tok.coord, -2.0f, 2.0f);
    auto radius = new Sampler!Tok("radius", Tok.radius, 0.02f, 0.06f);

    auto startRef = nt("startRef", [
        new Production([marker(Tok.refLast)]),
        new Production([idx]),
    ]);

    auto seedPos = nt("seedPos", [new Production([seedX, seedY, seedZ])]);

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

    auto start = nt("frame", [
        new Production([seedPos, beamList_]),
    ]);

    auto symbols = [
        start, seedPos, seedX, seedY, seedZ, beamList_, beam, startRef, idx,
        endRef, destX, destY, destZ, radius, beamKind,
    ];
    return new Grammar(start, symbols);
}

/**
 * Разобрать терминалы в правую половину каркаса.
 *
 * Создаётся seed-узел, затем балки по порядку. Токены `refLast` ссылаются
 * на последний созданный узел, `refIdx` — на существующий по номеру,
 * `endNew` (со смещениями) создаёт новый узел на позиции старта + смещение.
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
    while (i < tokens.length)
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
    return true;
}

/// Минимальная проверка каркаса: узлы в границах, без вырожденных балок,
/// весь каркас — один связный граф.
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

    bool[] visited = new bool[f.nodes.length];
    size_t[] stack = [0];
    visited[0] = true;
    while (stack.length > 0)
    {
        auto n = stack[$ - 1];
        stack.length -= 1;
        foreach (b; f.beams)
        {
            size_t next;
            if (b.a == n)
                next = b.b;
            else if (b.b == n)
                next = b.a;
            else
                continue;
            if (!visited[next])
            {
                visited[next] = true;
                stack ~= next;
            }
        }
    }
    foreach (v; visited)
        if (!v)
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
        }
    }

    // Декодирование сходится почти всегда; значительная доля геномов даёт
    // валидный каркас (остальные отбрасываются самокоррекцией — ссылки на
    // узлы, которых ещё нет, вырожденные балки, cross/axial вне оси).
    // С непрерывными самплерами доля валидных ~1.7% (см. counter).
    assert(decodeOk > 3000);
    assert(valid > 50);

    // Кроссинговер сохраняет число генов (по одному на нетерминал).
    auto g1 = randomGenotype(gr, 8, rnd);
    auto g2 = randomGenotype(gr, 8, rnd);
    auto child = crossover(g1, g2, rnd);
    assert(child.genes.length == gr.symbols.length);
}