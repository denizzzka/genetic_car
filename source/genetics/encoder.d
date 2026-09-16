module genetics.encoder;

import std.math : fabs;
import dlib.math.vector;
import frame.frame;
import genetics.sge;
import genetics.buggygrammar;

/**
 * Frame -> token plan (same terminals frameFromTokens expects).
 *
 * DFS from node 0 builds a spanning tree: tree edge -> endNew + delta,
 * back edge -> refIdx of both endpoints.  "last" tracks the creation-order
 * index of the most recently endNew'd node, same as in the decoder.
 */
Terminal!Tok[] frameToTokens(const Frame f)
in
{
    assert(f.nodes.length > 0);
}
do
{
    Terminal!Tok[] result;

    result ~= new Terminal!Tok(Tok.coord, f.nodes[0].pos.x);
    result ~= new Terminal!Tok(Tok.coord, f.nodes[0].pos.y);
    result ~= new Terminal!Tok(Tok.coord, f.nodes[0].pos.z);

    struct AdjEdge { size_t to; size_t beamIdx; }
    AdjEdge[][] adj;
    adj.length = f.nodes.length;

    foreach (bi, ref b; f.beams)
    {
        adj[b.a] ~= AdjEdge(b.b, bi);
        adj[b.b] ~= AdjEdge(b.a, bi);
    }

    bool[] visited = new bool[f.nodes.length];
    size_t[] order = new size_t[f.nodes.length];
    visited[0] = true;
    order[0] = 0;
    size_t created = 1;
    size_t last = 0;

    bool[] beamEmitted = new bool[f.beams.length];

    struct BackEdge { size_t from, to; size_t beamIdx; }
    BackEdge[] backEdges;

    void dfs(size_t u)
    {
        foreach (ref e; adj[u])
        {
            if (beamEmitted[e.beamIdx])
                continue;
            beamEmitted[e.beamIdx] = true;

            if (!visited[e.to])
            {
                visited[e.to] = true;
                order[e.to] = created++;

                if (last == order[u])
                    result ~= new Terminal!Tok(Tok.refLast);
                else
                    result ~= new Terminal!Tok(Tok.refIdx, cast(int) order[u]);

                result ~= new Terminal!Tok(Tok.endNew);
                auto delta = f.nodes[e.to].pos - f.nodes[u].pos;
                result ~= new Terminal!Tok(Tok.coord, delta.x);
                result ~= new Terminal!Tok(Tok.coord, delta.y);
                result ~= new Terminal!Tok(Tok.coord, delta.z);

                result ~= new Terminal!Tok(Tok.radius, f.beams[e.beamIdx].radius);
                result ~= new Terminal!Tok(Tok.beamKind, cast(int) f.beams[e.beamIdx].kind);

                last = order[e.to];
                dfs(e.to);
            }
            else
                backEdges ~= BackEdge(order[u], order[e.to], e.beamIdx);
        }
    }

    dfs(0);

    foreach (v; visited)
        assert(v, "frame must be connected from node 0");

    foreach (ref be; backEdges)
    {
        result ~= new Terminal!Tok(Tok.refIdx, cast(int) be.from);
        result ~= new Terminal!Tok(Tok.refIdx, cast(int) be.to);
        result ~= new Terminal!Tok(Tok.radius, f.beams[be.beamIdx].radius);
        result ~= new Terminal!Tok(Tok.beamKind, cast(int) f.beams[be.beamIdx].kind);
    }

    return result;
}

private size_t countBeams(const Terminal!Tok[] tokens, size_t start)
{
    size_t count;
    size_t i = start;
    while (i < tokens.length)
    {
        i++; // startRef
        if (tokens[i].tok == Tok.endNew)
            i += 4;
        else
            i++; // refIdx
        i++; // radius
        i++; // beamKind
        count++;
    }
    return count;
}

/**
 * Token plan -> Genotype.
 *
 * Walks the grammar derivation left-to-right (same order as decode) and
 * writes one codon per nonterminal expansion / sampler read.
 *
 * Production choice codon = production index (so codon % N == index).
 * Sampler float codon:  round((v - min)/(max-min) * uint.max).
 * Sampler int codon:    value itself (value < bound).
 */
Genotype encodeTokens(Grammar gr, const Terminal!Tok[] tokens)
{
    auto gt = new Genotype(gr.symbols.length);

    NonTerminal[string] ntMap;
    Sampler!Tok[string] samplerMap;
    foreach (sym; gr.symbols)
    {
        ntMap[sym.name] = sym;
        auto sp = cast(Sampler!Tok) sym;
        if (sp !is null)
            samplerMap[sym.name] = sp;
    }

    auto findNT(string name) { return ntMap[name]; }
    auto findSampler(string name) { return samplerMap[name]; }

    uint encodeFloat(float v, float min, float max)
    {
        double range = cast(double) max - cast(double) min;
        double t = (cast(double) v - cast(double) min) / range;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        return cast(uint)(t * cast(double) uint.max);
    }

    size_t pi;

    gt.genes[findNT("frame").id] ~= 0u;
    gt.genes[findNT("seedPos").id] ~= 0u;

    auto seedX = findSampler("seedX");
    auto seedY = findSampler("seedY");
    auto seedZ = findSampler("seedZ");
    gt.genes[seedX.id] ~= encodeFloat(tokens[pi++].f, seedX.min, seedX.max);
    gt.genes[seedY.id] ~= encodeFloat(tokens[pi++].f, seedY.min, seedY.max);
    gt.genes[seedZ.id] ~= encodeFloat(tokens[pi++].f, seedZ.min, seedZ.max);

    auto beamList_ = findNT("beamList");
    auto beam = findNT("beam");
    auto startRef = findNT("startRef");
    auto endRef = findNT("endRef");
    auto beamKind_ = findNT("beamKind");
    auto idx = findSampler("idx");
    auto destX = findSampler("destX");
    auto destY = findSampler("destY");
    auto destZ = findSampler("destZ");
    auto radius = findSampler("radius");

    size_t totalBeams = countBeams(tokens, pi);

    while (pi < tokens.length)
    {
        totalBeams--;
        gt.genes[beamList_.id] ~= (totalBeams > 0) ? 0u : 1u;
        gt.genes[beam.id] ~= 0u;

        if (tokens[pi].tok == Tok.refLast)
        {
            gt.genes[startRef.id] ~= 0u;
            pi++;
        }
        else
        {
            gt.genes[startRef.id] ~= 1u;
            gt.genes[idx.id] ~= cast(uint) tokens[pi].i;
            pi++;
        }

        if (tokens[pi].tok == Tok.endNew)
        {
            gt.genes[endRef.id] ~= 0u;
            pi++;
            gt.genes[destX.id] ~= encodeFloat(tokens[pi++].f, destX.min, destX.max);
            gt.genes[destY.id] ~= encodeFloat(tokens[pi++].f, destY.min, destY.max);
            gt.genes[destZ.id] ~= encodeFloat(tokens[pi++].f, destZ.min, destZ.max);
        }
        else
        {
            gt.genes[endRef.id] ~= 1u;
            gt.genes[idx.id] ~= cast(uint) tokens[pi].i;
            pi++;
        }

        gt.genes[radius.id] ~= encodeFloat(tokens[pi++].f, radius.min, radius.max);
        gt.genes[beamKind_.id] ~= cast(uint) tokens[pi++].i;
    }

    return gt;
}

Genotype encodeFrame(Grammar gr, const Frame f)
{
    auto tokens = frameToTokens(f);
    return encodeTokens(gr, tokens);
}

unittest
{
    import frame.buggy : buggyFrame;
    import std.random : Random;

    auto gr = buggyGrammar();
    const f = buggyFrame();

    auto tokens = frameToTokens(f);
    assert(tokens.length > 3);
    assert(tokens[0].tok == Tok.coord);
    assert(tokens[1].tok == Tok.coord);
    assert(tokens[2].tok == Tok.coord);

    auto gt = encodeTokens(gr, tokens);
    assert(gt.genes.length == gr.symbols.length);

    bool ok;
    auto decoded = decode!Tok(gr, gt, ok);
    assert(ok, "decode must succeed");
    assert(decoded.length == tokens.length);

    Frame f2;
    assert(frameFromTokens(decoded, f2), "frameFromTokens must succeed");
    assert(isValidFrame(f2), "decoded frame must be valid");
    assert(f.nodes.length == f2.nodes.length);
    assert(f.beams.length == f2.beams.length);

    // Точность: геометрия совпадает (учитываем перестановку индексов узлов,
    // вызванную порядком создания при декодировании), в пределах planeEpsilon.
    bool[] nodeUsed = new bool[f2.nodes.length];
    size_t[] idx = new size_t[f.nodes.length];
    foreach (i, n; f.nodes)
    {
        bool found;
        foreach (j, ref n2; f2.nodes)
        {
            if (nodeUsed[j])
                continue;
            if (distance(n.pos, n2.pos) <= planeEpsilon)
            {
                idx[i] = j;
                nodeUsed[j] = true;
                found = true;
                break;
            }
        }
        assert(found, "node not found in roundtrip");
    }

    bool[] beamUsed = new bool[f2.beams.length];
    foreach (b1; f.beams)
    {
        bool found;
        foreach (bi, b2; f2.beams)
        {
            if (beamUsed[bi])
                continue;
            const iA = idx[b1.a];
            const iB = idx[b1.b];
            const bool fwd = b2.a == iA && b2.b == iB;
            const bool rev = b2.a == iB && b2.b == iA;
            if ((fwd || rev) && fabs(b2.radius - b1.radius) <= planeEpsilon
                && b2.kind == b1.kind)
            {
                beamUsed[bi] = true;
                found = true;
                break;
            }
        }
        assert(found, "beam not found in roundtrip");
    }

    auto rnd = Random(1);
    mutate(gt, 1, rnd);
    assert(gt.genes.length == gr.symbols.length);
}

unittest
{
    import frame.buggy : buggyFrame;
    import std.random;

    auto gr = buggyGrammar();
    auto genome = encodeFrame(gr, buggyFrame());
    auto rnd = Random(3);

    // Мутация малого числа кодонов (1-3) в подавляющем большинстве случаев
    // должна оставаться валидной и не разрушать каркас целиком.
    size_t okCount;
    size_t beamTotal;
    foreach (_; 0 .. 200)
    {
        auto c = genome.dup;
        mutate(c, 1 + uniform(0u, 3u, rnd), rnd);
        bool ok;
        auto f = develop(gr, c, ok);
        if (!ok)
            continue;
        ++okCount;
        beamTotal += f.beams.length;
    }

    assert(okCount > 100, "большинство точечных мутаций должны развиваться в валидный каркас");
    const avg = beamTotal / okCount;
    assert(avg >= 10 && avg <= 70,
        "одна мутация не должна обрушивать или раздувать каркас");
}
