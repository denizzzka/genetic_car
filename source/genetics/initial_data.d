module genetics.initial_data;

// Стартовый геном эволюции: две балки подряд с общей средней нодой и два
// якоря, собранные в кодовоны напрямую.

import frame.frame;
import genetics.sge;
import genetics.buggygrammar;

/// Стартовый геном: две поперечные балки подряд («гироскутер» с нодой
/// посередине) и два якоря (колесо + моторное колесо) на внешних концах.
/// Цепочка из двух балок важна кодированием: у гена `beamList` больше одного
/// кодона, поэтому точечные мутации могут как укорачивать, так и наращивать
/// число балок, а средняя нода даёт точку ветвления. Дальше эволюция сама
/// добавит структуру, если это выгодно.
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

    // Seed — смещение точки старта от начала координат как
    // «единичный вектор направления × множитель длины»: тут вверх на 0.3 м.
    enum float startLen = 0.3f;
    const vSeed = up * startLen;
    set("startForward", [u(vSeed.x, -2.0f, 2.0f)]);
    set("startRight", [u(vSeed.y, -2.0f, 2.0f)]);
    set("startUp", [u(vSeed.z, -2.0f, 2.0f)]);
    set("taper", [u(1.0f, 0.4f, 1.0f)]);
    set("taperPow", [u(1.0f, 0.5f, 4.0f)]);
    set("heading", [mid]);
    set("motorPower", [u(initialMotorPower, -200.0f, 200.0f)]);

    // Один одиночный (медианный) сегмент без раздвоения: из него эволюция
    // либо вырастит пару ветвей (сегмент раздвоится — «лишняя пара
    // конечностей»), либо добавит ещё сегменты.
    set("segmentList", [1u]);
    set("segment", [0u]);
    set("segMode", [0u]);

    // Две балки подряд: `beamList = [0, 1]` — сначала «продолжить»
    // (продукция [beam, beamList]), затем «закончить» ([beam]). Обе растут
    // из последнего созданного узла: первая — из seed, вторая — из средней
    // ноды, поэтому `startRef` — refLast на оба кодирующих кодона.
    set("beamList", [0u, 1u]);
    set("beam", [0u]);
    set("startRef", [0u, 0u]);
    set("endRef", [0u, 0u]);

    // Дельта конца каждой балки — «единичный вектор направления × множитель
    // длины»: по половине поперечного пролёта (0.6 + 0.6 = 1.2 м суммарно,
    // на манер гироскутера), по ходу/вверх — ноль. По кодону на балку.
    enum float beamLen = 1.2f;
    const vBeam = left * (beamLen / 2.0f);
    set("forward", [u(vBeam.x, -1.5f, 1.5f), u(vBeam.x, -1.5f, 1.5f)]);
    set("right", [u(vBeam.y, -1.5f, 1.5f), u(vBeam.y, -1.5f, 1.5f)]);
    set("up", [u(vBeam.z, -1.5f, 1.5f), u(vBeam.z, -1.5f, 1.5f)]);
    set("radius", [u(0.05f, 0.02f, 0.06f), u(0.05f, 0.02f, 0.06f)]);
    // Активатор-ингибитор Nodal/Lefty на нуле: стартовая пара (когда
    // раздвоится) зеркально-точная; эволюция сама добавит асимметрию,
    // если это выгодно.
    set("nodal", [mid, mid]);
    set("lefty", [0u, 0u]);
    set("beamKind", [0u]);
    set("turn", [mid, mid]);

    set("anchorMarker", [0u]);
    set("anchorList", [0u, 1u]);
    set("anchor", [0u, 0u]);
    set("anchorKind", [0u, 1u]);
    // Якоря на внешних концах цепочки: seed — узел 0, конец второй балки — 2.
    set("idx", [0u, 2u]);

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
    // Две балки подряд: 0—1 и 1—2, средняя нода 1 общая.
    assert(f.nodes.length == 3);
    assert(f.beams.length == 2);
    assert(f.beams[0].a == 0 && f.beams[0].b == 1);
    assert(f.beams[1].a == 1 && f.beams[1].b == 2);
    assert(f.anchors.length == 2);
    assert(f.anchors[0].kind == AnchorKind.wheel && f.anchors[0].node == 0);
    assert(f.anchors[1].kind == AnchorKind.motorWheel && f.anchors[1].node == 2);
    assert(f.totalBeamLength > 0.0f);
    assert(f.motorPower > 99.0f && f.motorPower < 101.0f,
        "стартовая сила мотора берётся из гена motorPower");

    // Стартовый каркас — плоский и узкий, но морфологический суррогат не
    // должен его занулять: иначе всё нулевое поколение «проваливается» и
    // основатель не попадает даже в физику.
    import genetics.fitness : buggyFitness;
    assert(buggyFitness(f) > 0.0f,
        "статический фитнес стартового каркаса обязан быть больше нуля");

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