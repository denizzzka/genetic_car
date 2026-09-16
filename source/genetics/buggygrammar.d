module genetics.buggygrammar;

import dlib.math.vector;
import frame.frame;
import genetics.sge;

/// Семантический тип терминала, по нему фенотип-билдер разбирает токены.
enum Tok { anchor, index, coord, beamKind }

private Terminal!Tok t(Tok tok, int i = 0, float f = 0)
{
    return new Terminal!Tok(tok, i, f);
}

private NonTerminal nt(string name, Production[] productions)
{
    auto n = new NonTerminal(name);
    n.productions = productions;
    return n;
}

private Production[] nodeListOf(NonTerminal node, NonTerminal nodeList)
{
    return [
        new Production([node, nodeList]), // ещё узел
        new Production([node]),           // стоп
    ];
}

/**
 * Грамматика правой половины багги.
 *
 * Сначала порождается список узлов (позиция + тип якоря), затем список балок
 * (индексы узлов, радиус, тип). Позиция x >= 0; x == 0 означает узел на
 * плоскости симметрии.
 */
Grammar buggyGrammar()
{
    auto nodeList_ = nt("nodeList", null);
    auto beamList_ = nt("beamList", null);

    auto anchor = nt("anchor", [
        new Production([t(Tok.anchor, AnchorKind.none)]),
        new Production([t(Tok.anchor, AnchorKind.wheel)]),
        new Production([t(Tok.anchor, AnchorKind.wheelDrive)]),
        new Production([t(Tok.anchor, AnchorKind.motor)]),
        new Production([t(Tok.anchor, AnchorKind.shock)]),
        new Production([t(Tok.anchor, AnchorKind.spring)]),
        new Production([t(Tok.anchor, AnchorKind.axle)]),
    ]);

    auto xCoord = nt("x", [
        new Production([t(Tok.coord, 0, 0.00f)]),  // на оси
        new Production([t(Tok.coord, 0, 0.50f)]),  // вправо от оси
    ]);

    auto coord = nt("coord", [
        new Production([t(Tok.coord, 0, -1.0f)]),
        new Production([t(Tok.coord, 0, -0.5f)]),
        new Production([t(Tok.coord, 0, 0.0f)]),
        new Production([t(Tok.coord, 0, 0.5f)]),
        new Production([t(Tok.coord, 0, 1.0f)]),
    ]);

    auto idx = nt("idx", [
        new Production([t(Tok.index, 0)]),
        new Production([t(Tok.index, 1)]),
        new Production([t(Tok.index, 2)]),
        new Production([t(Tok.index, 3)]),
        new Production([t(Tok.index, 4)]),
    ]);

    auto radius = nt("radius", [
        new Production([t(Tok.coord, 0, 0.03f)]),
        new Production([t(Tok.coord, 0, 0.04f)]),
        new Production([t(Tok.coord, 0, 0.05f)]),
    ]);

    auto beamKind = nt("beamKind", [
        new Production([t(Tok.beamKind, BeamKind.normal)]),
        new Production([t(Tok.beamKind, BeamKind.cross)]),
        new Production([t(Tok.beamKind, BeamKind.axial)]),
    ]);

    auto node = nt("node", [
        new Production([anchor, xCoord, coord, coord]),
    ]);

    nodeList_.productions = [
        new Production([node, nodeList_]),
        new Production([node]),
    ];

    auto beam = nt("beam", [
        new Production([idx, idx, radius, beamKind]),
    ]);

    beamList_.productions = [
        new Production([beam, beamList_]),
        new Production([beam]),
    ];

    auto start = nt("frame", [
        new Production([nodeList_, beamList_]),
    ]);

    auto symbols = [
        start, nodeList_, node, anchor, xCoord, coord,
        idx, radius, beamKind, beam, beamList_,
    ];
    return new Grammar(start, symbols);
}

/**
 * Разобрать терминалы в правую половину каркаса.
 *
 * Возвращает false, если структура токенов не сошлась: в узле должны быть
 * x, y, z, в балке — две индекса, радиус и тип.
 */
bool frameFromTokens(const Terminal!Tok[] tokens, out Frame result)
{
    result = Frame.init;
    size_t i = 0;
    while (i < tokens.length)
    {
        switch (tokens[i].tok)
        {
            case Tok.anchor:
                if (i + 3 >= tokens.length)
                    return false;
                if (tokens[i + 1].tok != Tok.coord
                    || tokens[i + 2].tok != Tok.coord
                    || tokens[i + 3].tok != Tok.coord)
                    return false;
                auto x = tokens[i + 1].f;
                auto y = tokens[i + 2].f;
                auto z = tokens[i + 3].f;
                result.nodes ~= Node(vec3(x, y, z),
                    cast(AnchorKind) tokens[i].i);
                i += 4;
                break;
            case Tok.index:
                if (i + 3 >= tokens.length)
                    return false;
                if (tokens[i + 1].tok != Tok.index
                    || tokens[i + 2].tok != Tok.coord
                    || tokens[i + 3].tok != Tok.beamKind)
                    return false;
                result.beams ~= Beam(tokens[i].i, tokens[i + 1].i,
                    tokens[i + 2].f, cast(BeamKind) tokens[i + 3].i);
                i += 4;
                break;
            default:
                return false;
        }
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

    // Декодирование сходится почти всегда; случайная грамматика даёт и
    // валидные (связные) каркасы.
    assert(decodeOk > 3000);
    assert(valid > 0);

    // Кроссинговер сохраняет число генов (по одному на нетерминал).
    auto g1 = randomGenotype(gr, 8, rnd);
    auto g2 = randomGenotype(gr, 8, rnd);
    auto child = crossover(g1, g2, rnd);
    assert(child.genes.length == gr.symbols.length);
}
