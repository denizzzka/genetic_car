module genetics.initial_data;

// Стартовый геном эволюции: «гироскутер» — две балки от боковых точек
// скелета наружу, на внешних концах — два моторных якоря.

import std.math : abs;

import frame.frame;
import frame.cockpit : cockpitFrameNodeCount, cockpitFrameBeamCount;
import physics_world.wheel : defaultWheelRadius;
import genetics.sge;
import genetics.buggygrammar;

/// Стартовый геном: «гироскутер» — колёсная база из двух поперечных балок.
///
/// Старт медианный, раздвоения в нём нет: первая же раздвоенная сегментная
/// балка плодит дубликаты узлов и дубли якорей (каркас не viable), а от
/// боковых точек скелета раздвоение даёт пару лишь на месте самих точек.
/// Боковую симметрию эволюции обеспечивает заголовок, а не форма старта.
///
/// Turtle повёрнут на четверть оборота: его боковая ось есть мировая `right`,
/// а «вперёд» идёт вдоль `forward`, поэтому зеркальная плоскость заголовка
/// всегда поперёк машины — отдельная плоскость организма не нужна, и любое
/// будущее раздвоение сразу даёт пару лево/право.
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
    // startPos-гены (startForward/startRight/startUp) игнорируются: узел 0 —
    // всегда ЦМ кабины в начале координат frame.
    set("startForward", [mid]);
    set("startRight", [mid]);
    set("startUp", [mid]);
    set("taper", [u(1.0f, 0.4f, 1.0f)]);
    set("taperPow", [u(1.0f, 0.5f, 4.0f)]);
    // Четверть оборота: базис turtle совпадает с базисом frame — «вперёд»
    // идёт вдоль `forward`, боковая ось есть `right`, и зеркальная плоскость
    // сегмента всегда поперёк машины.
    enum float quarterTurn = 1.5707963f;
    set("heading", [u(-quarterTurn, -3.1416f, 3.1416f)]);
    set("motorPower", [u(initialMotorPower, -200.0f, 200.0f)]);
    // Организменный LR-градиент на нуле: тело стартует без полярности
    // лево/право, эволюция сама добавит её, если асимметрия окажется выгодной.
    set("lrGradient", [mid]);

    // Один медианный сегмент с двумя балками. У `segMode` по кодону на
    // сегмент, у `beamList` — по кодону на балку: 0 — «ещё балка»,
    // 1 — «конец списка».
    set("segmentList", [1u]);
    set("segment", [0u]);
    set("segMode", [0u]);

    // Балки — колёсная база: от боковых fix-точек средней станции (узлы 8 и
    // 9) наружу, концы — endNew, куда и вешаются моторные якоря.
    set("beamList", [0u, 1u]);
    set("beam", [0u]);
    set("startRef", [2u, 2u]);
    set("endRef", [0u, 0u]);

    enum float wheelOffset = 0.68f;
    set("forward", [mid, mid]);
    set("right", [u(+wheelOffset, -1.5f, 1.5f), u(-wheelOffset, -1.5f, 1.5f)]);
    set("up", [mid, mid]);
    set("radius", [u(0.05f, 0.02f, 0.06f), u(0.05f, 0.02f, 0.06f)]);
    // Активатор-ингибитор Nodal/Lefty на нуле: старт без асимметрии,
    // эволюция сама добавит её, если это выгодно.
    set("nodal", [mid, mid]);
    set("lefty", [0u, 0u]);
    // Множитель видовой мёртвой зоны — единица, то есть видовая норма:
    // отклик слабее 2 % асимметрии не материализуется.
    set("bilateralThreshold", [u(1.0f, 0.0f, 2.0f), u(1.0f, 0.0f, 2.0f)]);
    set("beamKind", [0u]);
    set("turn", [mid, mid]);

    set("anchorMarker", [0u]);
    set("anchorList", [0u, 1u]);
    set("anchor", [0u, 0u]);
    set("anchorKind", [1u, 1u]);
    // idx: адреса балок (8, 9). Значения 12 и 13 адресуют балки 3 и 4 при
    // круговом чтении гена — на этом держится рост колёсной базы наружу.
    set("idx", [8u, 9u, 12u, 13u]);
    // Адреса якорей — в своём гене, от количества балок не зависят.
    set("anchorIdx", [12u, 13u]);
    // Радиус колёс: оба якоря стартуют с `defaultWheelRadius`.
    set("wheelRadius", [u(defaultWheelRadius, 0.05f, 0.375f),
        u(defaultWheelRadius, 0.05f, 0.375f)]);

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
    // Гироскутер: скелет кабины и колёсная база (8 → 12, 9 → 13).
    assert(f.nodes.length == cockpitFrameNodeCount() + 2);
    assert(f.beams.length == cockpitFrameBeamCount() + 2);
    assert(f.beams[cockpitFrameBeamCount()].a == 8
        && f.beams[cockpitFrameBeamCount()].b == 12);
    assert(f.beams[cockpitFrameBeamCount() + 1].a == 9
        && f.beams[cockpitFrameBeamCount() + 1].b == 13);
    // Старт медианный: ни одна балка не отражена зеркально, симметрию задаёт
    // заголовок turtle, а не форма.
    foreach (b; f.beams[cockpitFrameBeamCount() .. $])
        assert(cast(Beam) b !is null,
            "стартовая балка — обычная, не эфемерная");
    assert(f.anchors.length == 2);
    assert(f.anchors.length == 2);
    assert(f.anchors[0].kind == AnchorKind.motorWheel && f.anchors[0].node == 12);
    assert(f.anchors[1].kind == AnchorKind.motorWheel && f.anchors[1].node == 13);
    assert(abs(f.anchors[0].radius - defaultWheelRadius) < 1e-6f
        && abs(f.anchors[1].radius - defaultWheelRadius) < 1e-6f,
        "оба стартовых колеса — заводского радиуса (30 см)");
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
    // Число балок живёт в узком диапазоне вокруг стартового (скелет + 2).
    const avg = beamTotal / okCount;
    assert(avg >= cockpitFrameBeamCount() && avg <= cockpitFrameBeamCount() + 6,
        "одна мутация не должна обрушивать или раздувать каркас");
}