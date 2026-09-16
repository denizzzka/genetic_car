module genetics.buggygrammar;

import std.math;
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
 * Узлы отдельно не генерируются: они появляются только как концы балок.
 * Каждая балка: старт — seed/последний созданный узел или существующий по
 * индексу, конец — вновь создаваемый (старт + смещение) или существующий
 * по индексу. Связность сохраняется, т.к. новые узлы прицепляются к старым.
 */
Grammar buggyGrammar()
{
    auto beamList_ = nt("beamList", null);

    auto idx = nt("idx", [
        new Production([t(Tok.refIdx, 0)]),
        new Production([t(Tok.refIdx, 1)]),
        new Production([t(Tok.refIdx, 2)]),
        new Production([t(Tok.refIdx, 3)]),
        new Production([t(Tok.refIdx, 4)]),
    ]);

    auto startRef = nt("startRef", [
        new Production([marker(Tok.refLast)]),
        new Production([idx]),
    ]);

    auto destX = nt("destX", [
        new Production([t(Tok.coord, 0.0f)]),
        new Production([t(Tok.coord, 0.35f)]),
        new Production([t(Tok.coord, 0.7f)]),
    ]);

    auto destY = nt("destY", [
        new Production([t(Tok.coord, -0.35f)]),
        new Production([t(Tok.coord, 0.0f)]),
        new Production([t(Tok.coord, 0.35f)]),
    ]);

    auto destZ = nt("destZ", [
        new Production([t(Tok.coord, 0.0f)]),
        new Production([t(Tok.coord, 0.35f)]),
        new Production([t(Tok.coord, 0.7f)]),
    ]);

    auto endRef = nt("endRef", [
        new Production([marker(Tok.endNew), destX, destY, destZ]),
        new Production([idx]),
    ]);

    auto radius = nt("radius", [
        new Production([t(Tok.radius, 0.03f)]),
        new Production([t(Tok.radius, 0.04f)]),
        new Production([t(Tok.radius, 0.05f)]),
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
        new Production([beamList_]),
    ]);

    auto symbols = [
        start, beamList_, beam, startRef, idx,
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

    result.nodes ~= Node(vec3(0.0f, 0.0f, 0.0f));
    size_t last = 0;

    size_t i = 0;
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
    foreach (b; f.beams)
    {
        if (b.a >= f.nodes.length || b.b >= f.nodes.length)
            return false;
        const pa = f.nodes[b.a].pos;
        const pb = f.nodes[b.b].pos;
        if (b.a == b.b || distance(pa, pb) < 1e-4f)
            return false;

        const aOnPlane = isOnPlane(pa);
        const bOnPlane = isOnPlane(pb);
        final switch (b.kind)
        {
            case BeamKind.normal:
                break;
            case BeamKind.cross:
                // Правый узел (вне плоскости) к осевому (x == 0),
                // на тех же y, z — иначе mirrorClosure упадёт.
                if (aOnPlane == bOnPlane)
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
/// ok=false если декодирование или разбор не удались.
Frame develop(const Grammar gr, const Genotype g, out bool ok)
{
    ok = false;
    auto tokens = decode!Tok(gr, g, ok);
    if (!ok)
        return Frame.init;
    Frame result;
    if (!frameFromTokens(tokens, result))
        return Frame.init;
    if (!isValidFrame(result))
        return Frame.init;
    ok = true;
    return result;
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

        bool ok;
        auto tokens = decode!Tok(gr, g, ok);
        if (!ok)
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
    assert(decodeOk > 3000);
    assert(valid > 250);

    // Кроссинговер сохраняет число генов (по одному на нетерминал).
    auto g1 = randomGenotype(gr, 8, rnd);
    auto g2 = randomGenotype(gr, 8, rnd);
    auto child = crossover(g1, g2, rnd);
    assert(child.genes.length == gr.symbols.length);
}