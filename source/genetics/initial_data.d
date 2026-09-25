module genetics.initial_data;

// Стартовый геном эволюции: «гироскутер» — две балки от боковых точек
// скелета наружу, на внешних концах — два моторных якоря.

import std.math : abs;

import dlib.math.vector;

import frame.frame;
import frame.cockpit : cockpitFrameNodeCount, cockpitFrameBeamCount;
import physics_world.wheel : defaultWheelRadius;
import genetics.sge;
import genetics.buggygrammar;

/// Стартовый геном: «гироскутер» — колёсная база из двух поперечных балок и
/// раздвоенная задняя спинка.
///
/// Базовые балки медианные: их зеркала — это и есть боковые точки скелета, а
/// раздвоение поверх них плодит дубликаты и дубли якорей (каркас не viable).
/// Спинка раздвоена и растёт из узла 5 — станции хребта, лежащей ровно в
/// зеркальной плоскости, поэтому раздвоение даёт пару поперёк машины. Балка
/// уходит назад и вбок, то есть за габарит кабины: внутри её бокового
/// габарита расти вверх нельзя. С этой пары эволюция мутирует дальше.
///
/// Turtle повёрнут на четверть оборота: его боковая ось есть мировая `right`,
/// а «вперёд» идёт вдоль `forward`, поэтому зеркальная плоскость заголовка
/// всегда поперёк машины — отдельная плоскость организма не нужна.
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

    // Два сегмента: первый медианный (колёсная база), второй раздвоенный
    // (спинка). У `segMode` по кодону на сегмент, у `beamList` — по кодону на
    // балку: 0 — «ещё балка», 1 — «конец списка».
    set("segmentList", [0u, 1u]);
    set("segment", [0u]);
    set("segMode", [0u, 1u]);

    // Балки 1 и 2 — колёсная база: от боковых fix-точек средней станции
    // (узлы 8 и 9) наружу, концы — endNew. Балка 3 — задняя спинка: старт от
    // последней станции хребта (узел 5) назад и вбок, то есть за габарит
    // кабины: внутри её бокового габарита расти вверх нельзя.
    set("beamList", [0u, 1u, 1u]);
    set("beam", [0u]);
    set("startRef", [2u, 2u, 2u]);
    set("endRef", [0u, 0u, 0u]);

    enum float wheelOffset = 0.68f;
    enum float tailBack = 0.6f;
    enum float tailSide = 0.4f;
    enum float tailUp = 0.2f;
    set("forward", [mid, mid, u(-tailBack, -1.5f, 1.5f)]);
    set("right", [u(+wheelOffset, -1.5f, 1.5f), u(-wheelOffset, -1.5f, 1.5f),
        u(+tailSide, -1.5f, 1.5f)]);
    set("up", [mid, mid, u(+tailUp, -1.5f, 1.5f)]);
    set("radius", [u(0.05f, 0.02f, 0.06f), u(0.05f, 0.02f, 0.06f),
        u(0.05f, 0.02f, 0.06f)]);
    // Активатор-ингибитор Nodal/Lefty на нуле: стартовая пара зеркальна
    // точно; эволюция сама добавит асимметрию, если это выгодно.
    set("nodal", [mid, mid, mid]);
    set("lefty", [0u, 0u, 0u]);
    // Множитель видовой мёртвой зоны — единица, то есть видовая норма:
    // отклик слабее 2 % асимметрии не материализуется.
    set("bilateralThreshold", [u(1.0f, 0.0f, 2.0f), u(1.0f, 0.0f, 2.0f),
        u(1.0f, 0.0f, 2.0f)]);
    set("beamKind", [0u]);
    set("turn", [mid, mid, mid]);

    set("anchorMarker", [0u]);
    set("anchorList", [0u, 1u]);
    set("anchor", [0u, 0u]);
    set("anchorKind", [1u, 1u]);
    // idx: старты балок (8, 9 и 5 — хвост), затем якоря на концах колёсной
    // базы (12, 13).
    set("idx", [8u, 9u, 5u, 12u, 13u]);
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
    // Гироскутер: скелет, колёсная база (8 → 12, 9 → 13) и раздвоенная
    // задняя спинка (5 → 14 с зеркалом 5 → 15).
    assert(f.nodes.length == cockpitFrameNodeCount() + 4);
    assert(f.beams.length == cockpitFrameBeamCount() + 4);
    assert(f.beams[cockpitFrameBeamCount()].a == 8
        && f.beams[cockpitFrameBeamCount()].b == 12);
    assert(f.beams[cockpitFrameBeamCount() + 1].a == 9
        && f.beams[cockpitFrameBeamCount() + 1].b == 13);
    assert(f.beams[cockpitFrameBeamCount() + 2].a == 5
        && f.beams[cockpitFrameBeamCount() + 2].b == 14
        && f.beams[cockpitFrameBeamCount() + 3].a == 5
        && f.beams[cockpitFrameBeamCount() + 3].b == 15,
        "спинка раздвоена: вторая балка — зеркало первой");
    const vec3 pair = f.nodes[14].pos - f.nodes[15].pos;
    assert(abs(dot(pair, right)) > 0.1f && abs(dot(pair, forward)) < 1e-4f
        && abs(dot(pair, up)) < 1e-4f,
        "пара спинки расходится строго вбок, симметрично плоскости хребта");
    assert(dot(f.nodes[14].pos, forward) < dot(f.nodes[5].pos, forward) - 0.1f
        && dot(f.nodes[14].pos, right) * dot(f.nodes[15].pos, right) < 0.0f,
        "спинка уходит назад от корня и в разные стороны");
    assert(cast(Beam) f.beams[cockpitFrameBeamCount()] !is null
        && cast(Beam) f.beams[cockpitFrameBeamCount() + 1] !is null,
        "обе стартовые балки обычные, не эфемерные");
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