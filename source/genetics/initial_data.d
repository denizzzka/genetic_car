module genetics.initial_data;

// Стартовый геном эволюции: одна балка и два якоря, собранные в кодовоны
// напрямую, без кодирования из готового фрейма.

import frame.frame;
import genetics.sge;
import genetics.buggygrammar;

/// Стартовый геном: одна балка от носа до кормы и два якоря (колесо +
/// моторное колесо). Дальше эволюция сама добавит структуру, если это
/// выгодно.
Genotype startGenome(const Grammar gr)
{
    auto gt = new Genotype(gr.symbols.length);

    uint u(float v, float mn, float mx)
    {
        double t = (cast(double) v - mn) / (cast(double) mx - mn);
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        return cast(uint)(t * cast(double) uint.max);
    }

    auto symId = (string name) {
        foreach (sym; gr.symbols)
            if (sym.name == name)
                return sym.id;
        assert(false);
    };
    auto set = (string name, uint[] vals) {
        gt.genes[symId(name)] = vals;
    };

    const mid = cast(uint)(uint.max / 2);

    set("frame", [0u]);
    set("startPos", [0u]);
    set("startX", [u(0.0f, -2.0f, 2.0f)]);
    set("startY", [u(0.0f, -2.0f, 2.0f)]);
    set("startZ", [u(0.3f, -2.0f, 2.0f)]);
    set("taper", [u(1.0f, 0.4f, 1.0f)]);
    set("taperPow", [u(1.0f, 0.5f, 4.0f)]);
    set("heading", [mid]);

    // Один одиночный (медианный) сегмент без раздвоения: из него эволюция
    // либо вырастит пару ветвей (сегмент раздвоится — «лишняя пара
    // конечностей»), либо добавит ещё сегменты.
    set("segmentList", [1u]);
    set("segment", [0u]);
    set("segMode", [0u]);

    set("beamList", [1u]);
    set("beam", [0u]);
    set("startRef", [0u]);
    set("endRef", [0u]);
    set("destX", [mid]);
    set("destY", [u(-1.2f, -1.5f, 1.5f)]);
    set("destZ", [mid]);
    set("radius", [u(0.05f, 0.02f, 0.06f)]);
    // Активатор-ингибитор Nodal/Lefty на нуле: стартовая пара (когда
    // раздвоится) зеркально-точная; эволюция сама добавит асимметрию,
    // если это выгодно.
    set("nodal", [mid]);
    set("lefty", [0u]);
    set("beamKind", [0u]);
    set("turn", [mid]);

    set("anchorMarker", [0u]);
    set("anchorList", [0u, 1u]);
    set("anchor", [0u, 0u]);
    set("anchorKind", [0u, 1u]);
    set("idx", [0u, 1u]);

    return gt;
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
    const f = may.get.frame;
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
        beamTotal += may.get.frame.beams.length;
    }

    assert(okCount > 100, "большинство точечных мутаций должны развиваться в валидный каркас");
    // Число балок живёт в узком диапазоне вокруг стартового.
    const avg = beamTotal / okCount;
    assert(avg >= 1 && avg <= 6,
        "одна мутация не должна обрушивать или раздувать каркас");
}