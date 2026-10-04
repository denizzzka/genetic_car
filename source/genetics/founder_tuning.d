module genetics.founder_tuning;

import std.algorithm : max, min;
import std.array : array;
import std.conv : to;
import std.format : format;
import std.math : abs;
import std.stdio : writeln;

import dlib.math.vector : distance, vec3;

import frame.frame : Frame;
import genetics.chromosome : Chromosome;
import genetics.fitness : buggyFitness, wheelAnchorSpacing;
import genetics.growth : develop;

/**
 * Подбор калибровки основателя вида.
 *
 * Механизм роста общий для всех особей, а вот исходная форма задаётся
 * дефолтами `Chromosome.init`. Их нужно не угадывать, а выбирать: этот модуль
 * прогоняет набор проб, мерит форму каждой и печатает отчёт, из которого видно,
 * какая калибровка даёт машину нужного облика.
 *
 * Ориентир — не мотоцикл, а гироскутер: поперечная балка через середину
 * машины, колёса по краям поперёк. Критерий `isGyroscooter` проверяет ровно
 * это, остальное — производные.
 */

/**
 * Порог «колесо считается боковым». Ширина кабины — около метра, точки
 * крепления стоят на |x| = 0.3, поэтому колесо на |x| > sideWheelX уже
 * висит сбоку от корпуса, а не под ним.
 */
enum float sideWheelX = 0.3f;

/**
 * Насколько колёса могут отличаться по курсу, чтобы это ещё читалось как
 * «колёса рядом», а не как мотоцикл. В гейт не входит: форма может быть
 * кривой, важно лишь что колёса по бокам. Значение нужно отчёту и сортировке.
 */
enum float axleYSpan = 1.2f;

/// Одна проба: хромосома, выросший из неё каркас и всё, что о нём можно
/// сказать, не запуская физику.
struct FounderProbe
{
    Chromosome chr;

    /// `false` — каркас не вырос вовсе: полем не нашлось места ни для двух
    /// колёс, и машина не поедет принципиально.
    bool grew;

    size_t nodes;
    size_t beams;
    size_t anchors;
    size_t sideAnchors;

    /// Разброс колёс поперёк: `xmin .. xmax`.
    float xmin = float.max;
    float xmax = -float.max;

    /// Разброс колёс по курсу: `ymin .. ymax`.
    float ymin = float.max;
    float ymax = -float.max;

    /// Разброс колёс по высоте: `zmin .. zmax`.
    float zmin = float.max;
    float zmax = -float.max;

    /// Зазор между ободами, м. Отрицательный — колёса наезжают.
    float gap;

    /// Статический фитнес: 0 — машина негодная.
    float fitness;

    /// Поперечина соединяет ведущее и ведомое колесо напрямую.
    bool axleBeam;
}

/// Вырастить каркас из хромосомы и обмерить его.
FounderProbe probe(const Chromosome chr)
{
    FounderProbe p;
    p.chr = chr;

    const auto grown = develop(chr);
    if (grown.isNull)
        return p;

    p.grew = true;
    const auto f = grown.get;
    p.nodes = f.nodes.length;
    p.beams = f.beams.length;
    p.anchors = f.anchors.length;
    p.gap = wheelAnchorSpacing(f);
    p.fitness = buggyFitness(f);

    foreach (a; f.anchors)
    {
        const vec3 pos = f.nodes[a.node].pos;
        p.xmin = min(p.xmin, pos.x);
        p.xmax = max(p.xmax, pos.x);
        p.ymin = min(p.ymin, pos.y);
        p.ymax = max(p.ymax, pos.y);
        p.zmin = min(p.zmin, pos.z);
        p.zmax = max(p.zmax, pos.z);
        if (abs(pos.x) > sideWheelX)
            ++p.sideAnchors;
    }
    p.axleBeam = hasAxleBeam(f);
    return p;
}

/// Прямая балка между двумя боковыми колёсами — то самое «балка по ширине».
private bool hasAxleBeam(const Frame f)
{
    size_t[] side;
    foreach (a; f.anchors)
        if (abs(f.nodes[a.node].pos.x) > sideWheelX)
            side ~= a.node;
    if (side.length < 2)
        return false;
    foreach (i; 0 .. side.length)
        foreach (j; i + 1 .. side.length)
            foreach (b; f.beams)
                if ((b.a == side[i] && b.b == side[j]) || (b.a == side[j] && b.b == side[i]))
                    return true;
    return false;
}

/// Гироскутер ли это: колёса по обе стороны корпуса и машина при этом едетная.
///
/// Кривизна балок, длина ветвей и разброс колёс по курсу не проверяются:
/// важно, что колёса висят сбоку, а не под корпусом и не по центру. Среди
/// гироскутеров `bestGyroscooter` сам выбирает ровнее — по фитнесу.
bool isGyroscooter(const FounderProbe p)
{
    return p.grew && p.fitness > 0.0f && p.sideAnchors >= 2
        && p.xmin < -sideWheelX && p.xmax > sideWheelX;
}

/// Колёса стоят рядом по курсу (для отчёта и выбора лучшей пробы).
bool wheelsBeside(const FounderProbe p)
{
    return (p.ymax - p.ymin) <= axleYSpan;
}

/// Однострочный отчёт по пробе: форма, размер и годность.
string report(const FounderProbe p)
{
    if (!p.grew)
        return "не вырос";
    return format("узлов=%d балок=%d якорей=%d сбоку=%d gap=%.3f fit=%.3f "
        ~ "x=[%.2f..%.2f] y=[%.2f..%.2f] z=[%.2f..%.2f] рядом=%s поперечина=%s%s",
        p.nodes, p.beams, p.anchors, p.sideAnchors, p.gap, p.fitness,
        p.xmin, p.xmax, p.ymin, p.ymax, p.zmin, p.zmax,
        wheelsBeside(p) ? "да" : "нет", p.axleBeam ? "есть" : "нет",
        isGyroscooter(p) ? " ГИРОСКУТЕР" : "");
}

/// Оси подбора. Активатор задаёт, сколько тела вообще вырастет, ингибитор —
/// где оно остановится, поток — сторону перекоса. Сетка намеренно широкая:
/// калибровка выбирается по форме, а не угадывается.
private immutable float[] scanFlow = [0.0f, 0.15f, -0.15f];
private immutable float[] scanActProduction = [0.8f, 1.2f, 1.5f, 2.0f];
private immutable float[] scanInhProduction = [0.05f, 0.1f, 0.2f, 0.35f];

/**
 * Прогнать калибровки основателя по осям подбора и напечатать отчёт.
 *
 * Каждая проба — все гены основателя, кроме перебираемых. Возвращает пробы
 * для отчёта и для тестов, печать — побочный эффект.
 */
FounderProbe[] scanFounder(Chromosome founder = Chromosome.init,
    bool print = true)
{
    FounderProbe[] probes;
    foreach (flow; scanFlow)
    foreach (actProduction; scanActProduction)
    foreach (inhProduction; scanInhProduction)
    {
        Chromosome chr = founder;
        chr.flowStrength = flow;
        chr.actProduction = actProduction;
        chr.inhProduction = inhProduction;
        const auto p = probe(chr);
        probes ~= p;
        if (print)
            writeln(format("flow=%+.2f actP=%.1f inhP=%.2f -> %s",
                flow, actProduction, inhProduction, report(p)));
    }
    return probes;
}

/// Лучшая гироскутерная проба из набора: с максимальным фитнесом среди тех,
/// что действительно гироскутер.
bool bestGyroscooter(const FounderProbe[] probes, out FounderProbe best)
{
    bool found;
    foreach (p; probes)
        if (isGyroscooter(p) && (!found || p.fitness > best.fitness))
        {
            best = p;
            found = true;
        }
    return found;
}

unittest
{
    // Модуль ничего не выращивает сам: проба основателя обязана получиться.
    const auto p = probe(Chromosome.init);
    assert(p.grew, "основатель обязан вырасти в каркас");

    // Отчёт не пустой ни для годной, ни для провалившейся пробы.
    assert(report(p).length > 0);
    assert(report(FounderProbe.init) == "не вырос");

    // Критерий гироскутера: колёса по обе стороны корпуса. Кривизна и разброс
    // по курсу в него не входят — они лишь сортируют пробы между собой.
    FounderProbe fake;
    fake.grew = true;
    fake.fitness = 1.0f;
    fake.sideAnchors = 2;
    fake.xmin = -0.9f;
    fake.xmax = 0.9f;
    fake.ymin = -0.1f;
    fake.ymax = 0.1f;
    assert(isGyroscooter(fake), "боковые колёса по обе стороны — гироскутер");
    assert(wheelsBeside(fake), "колёса на одной станции стоят рядом по курсу");
    fake.ymax = 1.4f;
    assert(isGyroscooter(fake), "разброс по курсу гироскутером быть не мешает");
    assert(!wheelsBeside(fake), "колёса, разнесённые по курсу, не рядом");
    fake.ymax = 0.1f;
    fake.fitness = 0.0f;
    assert(!isGyroscooter(fake), "негодная машина гироскутером не считается");
}