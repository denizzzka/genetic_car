module genetics.encoder;

import dlib.math.vector;
import frame.frame;
import genetics.sge;
import genetics.buggygrammar;

/**
 * Frame -> token plan (same terminals frameFromTokens expects).
 *
 * BFS from node 0 builds a spanning tree: tree edge -> endNew + delta,
 * back edge -> refIdx of both endpoints.  "last" tracks the creation-order
 * index of the most recently endNew'd node, same as in the decoder.
 * BFS (а не DFS) выбирает короткие рёбра остовного дерева, чтобы дельты
 * попадали в диапазон самплеров destX/destY/destZ.
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

    // Обход в ширину вместо глубины: BFS-дерево обходит узлы по коротким
    // рёбрам, и дельты (u -> v) почти всегда попадают в диапазон самплеров
    // destX/destY/destZ (иначе длинное ребро остовного дерева не удалось бы
    // закодировать, и roundtrip потерял бы геометрию).
    size_t[] queue = [0];
    while (queue.length > 0)
    {
        const u = queue[0];
        queue = queue[1 .. $];

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
                queue ~= e.to;
            }
            else
                backEdges ~= BackEdge(order[u], order[e.to], e.beamIdx);
        }
    }

    foreach (v; visited)
        assert(v, "frame must be connected from node 0");

    foreach (ref be; backEdges)
    {
        result ~= new Terminal!Tok(Tok.refIdx, cast(int) be.from);
        result ~= new Terminal!Tok(Tok.refIdx, cast(int) be.to);
        result ~= new Terminal!Tok(Tok.radius, f.beams[be.beamIdx].radius);
        result ~= new Terminal!Tok(Tok.beamKind, cast(int) f.beams[be.beamIdx].kind);
    }

    // Якоря (колёса) — после всех балок. Индекс узла кодируется в порядке
    // создания нод декодером (`order[]`), а не в исходной нумерации каркаса.
    result ~= new Terminal!Tok(Tok.anchors);
    foreach (anchor; f.anchors)
    {
        result ~= new Terminal!Tok(Tok.anchorKind, cast(int) anchor.kind);
        result ~= new Terminal!Tok(Tok.refIdx, cast(int) order[anchor.node]);
    }

    return result;
}

private size_t countBeams(const Terminal!Tok[] tokens, size_t start)
{
    size_t count;
    size_t i = start;
    while (i < tokens.length && tokens[i].tok != Tok.anchors)
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

private size_t countAnchors(const Terminal!Tok[] tokens, size_t start)
{
    // `start` указывает на маркер Tok.anchors; дальше — пары anchorKind+refIdx.
    size_t count;
    size_t i = start + 1;
    while (i < tokens.length)
    {
        i += 2;
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
    gt.genes[findNT("startPos").id] ~= 0u;

    auto startX = findSampler("startX");
    auto startY = findSampler("startY");
    auto startZ = findSampler("startZ");
    assert(tokens[pi].tok == Tok.coord, "позиция первого узла: x");
    gt.genes[startX.id] ~= encodeFloat(tokens[pi++].f, startX.min, startX.max);
    assert(tokens[pi].tok == Tok.coord, "позиция первого узла: y");
    gt.genes[startY.id] ~= encodeFloat(tokens[pi++].f, startY.min, startY.max);
    assert(tokens[pi].tok == Tok.coord, "позиция первого узла: z");
    gt.genes[startZ.id] ~= encodeFloat(tokens[pi++].f, startZ.min, startZ.max);

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

    auto anchorMarker = findNT("anchorMarker");
    auto anchorList_ = findNT("anchorList");
    auto anchor = findNT("anchor");
    auto anchorKind_ = findNT("anchorKind");

    size_t totalBeams = countBeams(tokens, pi);

    while (pi < tokens.length && tokens[pi].tok != Tok.anchors)
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

    // Якоря: маркер, затем по паре (anchorKind, refIdx) на колесо.
    if (pi < tokens.length)
    {
        assert(tokens[pi].tok == Tok.anchors, "ожидался маркер начала якорей");
        gt.genes[anchorMarker.id] ~= 0u;
        pi++;

        size_t totalAnchors = countAnchors(tokens, pi - 1);
        while (pi < tokens.length)
        {
            totalAnchors--;
            gt.genes[anchorList_.id] ~= (totalAnchors > 0) ? 0u : 1u;
            gt.genes[anchor.id] ~= 0u;

            assert(tokens[pi].tok == Tok.anchorKind, "ожидался тип якоря");
            gt.genes[anchorKind_.id] ~= cast(uint) tokens[pi].i;
            pi++;

            assert(tokens[pi].tok == Tok.refIdx, "ожидался индекс узла якоря");
            gt.genes[idx.id] ~= cast(uint) tokens[pi].i;
            pi++;
        }
    }

    return gt;
}

Genotype encodeFrame(Grammar gr, const Frame f)
{
    auto tokens = frameToTokens(f);
    return encodeTokens(gr, tokens);
}

/// Простейший стартовый геном: одна балка, колесо на одном конце, моторное
/// колесо на другом
Genotype startGenome(Grammar gr)
{
    Frame f;
    size_t node(vec3 pos)
    {
        f.nodes ~= Node(pos);
        return f.nodes.length - 1;
    }

    const wheel = node(vec3(0.6f, 0.7f, 0.3f));
    const motor = node(vec3(0.6f, -0.7f, 0.3f));

    f.beams ~= Beam(wheel, motor, 0.05f);
    f.anchors ~= Anchor(wheel, AnchorKind.wheel);
    f.anchors ~= Anchor(motor, AnchorKind.motorWheel);

    return encodeFrame(gr, f);
}

unittest
{
    import std.random : Random;

    auto gr = buggyGrammar();
    auto genome = startGenome(gr);
    assert(genome.genes.length == gr.symbols.length);

    // Стартовая хромосома развивается в простейший каркас.
    auto may = develop(gr, genome);
    assert(!may.isNull, "стартовая хромосома должна развиваться");
    const f = may.get;
    assert(f.nodes.length == 2);
    assert(f.beams.length == 1);
    assert(f.anchors.length == 2);
    assert(f.anchors[0].kind == AnchorKind.wheel && f.anchors[0].node == 0);
    assert(f.anchors[1].kind == AnchorKind.motorWheel && f.anchors[1].node == 1);
    assert(f.totalBeamLength > 0.0f);

    auto rnd = Random(1);
    mutate(genome, 1, rnd);
    assert(genome.genes.length == gr.symbols.length);
}

unittest
{
    import std.random;

    auto gr = buggyGrammar();
    auto genome = startGenome(gr);
    auto rnd = Random(3);

    // Мутация малого числа кодонов (1-3) в подавляющем большинстве случаев
    // должна оставаться валидной и не разрушать каркас целиком.
    size_t okCount;
    size_t beamTotal;
    foreach (_; 0 .. 200)
    {
        auto c = genome.dup;
        mutate(c, 1 + uniform(0u, 3u, rnd), rnd);
        auto may = develop(gr, c);
        if (may.isNull)
            continue;
        ++okCount;
        beamTotal += may.get.beams.length;
    }

    assert(okCount > 100, "большинство точечных мутаций должны развиваться в валидный каркас");
    // Число балок живёт в узком диапазоне вокруг стартового.
    const avg = beamTotal / okCount;
    assert(avg >= 1 && avg <= 6,
        "одна мутация не должна обрушивать или раздувать каркас");
}

unittest
{
    import std.random: Random;

    // Число балок и колёс не заложено в стартовый каркас: за 50 шагов
    // мутации структура должна уметь вырасти, а не только менять геометрию.
    auto gr = buggyGrammar();
    auto genome = startGenome(gr);
    auto rnd = Random(42);

    const startBeams = develop(gr, genome).get.beams.length;
    const startAnchors = develop(gr, genome).get.anchors.length;
    bool grewBeams, grewAnchors;

    foreach (_; 0 .. 50)
    {
        auto next = mutateStep(gr, genome, rnd);
        if (next.isNull)
            continue;

        auto f = develop(gr, next.get).get;
        if (f.beams.length > startBeams)
            grewBeams = true;
        if (f.anchors.length > startAnchors)
            grewAnchors = true;
        genome = next.get;
    }

    assert(grewBeams, "балки должны уметь появляться");
    assert(grewAnchors, "колёса должны уметь появляться");
}