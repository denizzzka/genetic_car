module genetics.chromosome;

import std.algorithm : clamp, max, min;
import std.math : abs, cos, isFinite, log, sqrt, PI;
import std.random : Random, uniform;
import std.traits : FieldNameTuple;

import frame.frame : initialMotorPower;
import physics_world.wheel : defaultWheelRadius;

/// Наименьшее отношение радиуса ингибитора к радиусу активатора. Ниже — система
/// вырождается: ингибитор глушит собственную точку, и рост не идёт дальше
/// первого шага.
enum float inhSpreadFloor = 1.5f;

/// Границы числа фаз роста: без фаз тело не строится вовсе, а слишком длинный
/// список фаз — это уже не признак, а мусор.
enum size_t minSegments = 1;
enum size_t maxSegments = 8;

/// Вероятность индела — вставки или удаления фазы за одну мутацию. Так список
/// фаз становится признаком переменной длины, а не только набором значений.
enum float indelRate = 0.1f;

/// Гены вида: одно значение на организм, фазами не дробятся.
struct SpeciesGenes
{
    /// Радиус трубы новой балки, м.
    float beamRadius = 0.045f;

    /// Радиус колеса якоря, м.
    float wheelRadius = defaultWheelRadius;

    /// Сила мотор-колёс, Н·м; знак задаёт направление привода.
    float motorPower = initialMotorPower;
}

/// Гены фазы роста: поле, которым в этой фазе строится тело. Список таких фаз и
/// есть геном переменной длины — длина списка задаёт, сколько у машины фаз.
struct SegmentGenes
{
    /// Скорость роста активатора: насколько быстро активатор размножает себя
    /// там, где он уже есть. Мало — рост вялый, много — форма плывёт.
    float actProduction = 1.1f;

    /// Скорость роста ингибитора: насколько сильно активатор включает тормоз.
    /// Мало — рост бесконтрольный, много — комок у семени.
    float inhProduction = 0.1f;

    /// Радиус действия активатора: как далеко он дотягивается от своей точки.
    /// Работает локально, поэтому радиус мал.
    float actDiffusion = 0.32f;

    /// Радиус действия ингибитора: разносится далеко и глушит периферию.
    /// Всегда шире активатора — иначе паттерн не возникает, и всё превращается
    /// в однородную кашу. Это и есть суть Nodal/Lefty: быстрый локальный
    /// «включи» плюс медленный глобальный «выключи».
    float inhDiffusion = 1.2f;

    /// Порог постановки балки: при каком act − inh рост продолжается.
    /// Высокий — каркас разреженный, низкий — плотный.
    float threshold = 0.42f;

    /// Сила потока узла (nodal flow): уводит активатор вперёд по курсу.
    /// Знак задаёт, растёт фаза вперёд или назад. Ноль — роста по курсу нет.
    float flowStrength = -0.15f;

    /// Организатор средней линии: тянет ткань от срединной плоскости в стороны.
    /// Без него поле всего сильнее там, где тело уже есть, и ветви завираются
    /// внутрь. Малое значение фазы позволяет вести балку по курсу, не
    /// растекаясь вбок.
    float midlineShare = 16.0f;

    /// Базовый шаг новой балки, м: дискретизация пространства и её минимальная
    /// длина — без него поля нет, были бы точки, а не отрезки. Фактическая
    /// длина больше и задаётся локальным полем (см. genetics.growth).
    float stepLength = 0.18f;

    /// Сколько балок фаза может нарастить. Размер — тоже свойство фазы: иначе
    /// поле всегда доращивает каркас до предела, и отбор лишён выбора между
    /// компактной машиной и раздутой.
    float beamBudget = 2.0f;

    /// Скорость роста активатора моторного поля.
    float motorProduction = 0.2f;

    /// Радиус действия активатора моторного поля.
    float motorDiffusion = 2.5f;
}

/// Число генов выводится из полей, а не выписывается руками.
enum size_t speciesGeneCount = FieldNameTuple!SpeciesGenes.length;
enum size_t segmentGeneCount = FieldNameTuple!SegmentGenes.length;

/**
 * Хромосома — контейнер генов особи: гены вида плюс список фаз роста. Ген
 * здесь не «балка в точке», а правило роста: каркас вырастает сам из
 * реакционно-диффузионной системы Nodal/Lefty, по одной фазе за раз
 * (см. genetics.growth). Длина списка фаз — эволюционируемый признак.
 *
 * Значения по умолчанию — откалиброванный основатель вида.
 */
struct Chromosome
{
    SpeciesGenes species;

    SegmentGenes[] segments = founderSegments;

    alias species this;

    /// Глубокая копия: список фаз — ссылка, поэтому обычное присвоение копирует
    /// её лишь на вид, а `const`-хромосому нельзя присвоить переменной. Копия
    /// отвязывает фазы и годится для правки.
    Chromosome clone() const
    {
        Chromosome r;
        r.species = species;
        r.segments = segments.dup;
        return r;
    }
}

/**
 * Фазы основателя: стойка руля, поперечина, корпус. Первая фаза подавляет
 * организатор средней линии и гонит поток вперёд — из носа выходит одна
 * балка. Вторая возвращает боковую тягу и отпускает ступице два уса. Третья
 * растит остальное тело и несёт моторное поле.
 */
enum SegmentGenes[] founderSegments = [
    SegmentGenes(1.2f, 0.10f, 0.32f, 1.20f, 0.42f, 2.00f, 0.0f, 0.08f, 1.0f, 0.0f, 2.5f),
    SegmentGenes(1.2f, 0.10f, 0.12f, 0.30f, 0.42f, 0.00f, 20.0f, 0.10f, 4.0f, 0.2f, 2.5f),
    SegmentGenes(0.2f, 0.10f, 0.20f, 1.20f, 0.10f, 0.30f, 2.0f, 0.30f, 2.0f, 0.0f, 2.5f),
];

/// Границы генов вида — те же поля в том же порядке, что и `tupleof`.
enum float[speciesGeneCount] speciesLo = [0.02f, 0.10f, -200.0f];
enum float[speciesGeneCount] speciesHi = [0.08f, 0.40f, 200.0f];

/// Границы генов фазы — тоже свойства вида, а не гены: выход за границу
/// означает «не такая машина», а не другой организм.
enum float[segmentGeneCount] segmentLo = [
    0.20f, 0.05f, 0.10f, 0.30f, 0.02f, -2.00f, 0.00f, 0.08f,
    0.00f, 0.00f, 0.10f,
];
enum float[segmentGeneCount] segmentHi = [
    4.00f, 1.00f, 0.80f, 4.00f, 1.50f, 2.00f, 60.0f, 0.45f,
    64.00f, 2.00f, 3.00f,
];

// Границы обязаны быть заданы у каждого гена и быть конечными: список короче
// числа генов D дополняет NaN, а вырожденный диапазон намертво замораживает ген
// в нормализации. Ловим это здесь, а не в эволюции, потерявшей ген.
static foreach (i; 0 .. speciesGeneCount)
{
    static assert(speciesHi[i] > speciesLo[i],
        "у гена вида вырожденный диапазон: не заданы границы");
    static assert(isFinite(speciesLo[i]) && isFinite(speciesHi[i]),
        "границы гена вида не конечны: список короче числа генов");
}
static foreach (i; 0 .. segmentGeneCount)
{
    static assert(segmentHi[i] > segmentLo[i],
        "у гена фазы вырожденный диапазон: не заданы границы");
    static assert(isFinite(segmentLo[i]) && isFinite(segmentHi[i]),
        "границы гена фазы не конечны: список короче числа генов");
}

/// Зажать поля структуры по границам: один и тот же обход и для вида, и для
/// фазы, поэтому он параметризован типом.
private void clampFields(T)(ref T g, const float[] lo, const float[] hi)
{
    static foreach (i; 0 .. T.tupleof.length)
        g.tupleof[i] = clamp(g.tupleof[i], lo[i], hi[i]);
}

/// Хромосома в допустимых диапазонах: значения зажаты по генам, а ингибитор
/// оставлен шире активатора в каждой фазе. Правило держит и мутацию, и
/// кроссовер — иначе особь с негодным соотношением радиусов не вырастет.
Chromosome normalized(const Chromosome c)
{
    auto r = c.clone;
    if (r.segments.length < minSegments)
        r.segments = [SegmentGenes.init];
    clampFields(r.species, speciesLo, speciesHi);
    foreach (ref seg; r.segments)
    {
        clampFields(seg, segmentLo, segmentHi);
        seg.inhDiffusion = max(seg.inhDiffusion, seg.actDiffusion * inhSpreadFloor);
    }
    return r;
}

/// Мутация: гауссов шум по каждому гену, шум измеряется долей диапазона гена,
/// плюс индел — вставка или удаление фазы. Одна мутация меняет все гены разом,
/// но слабо: организм остаётся собой, а число фаз дрейфует.
Chromosome mutated(const Chromosome c, float rate, ref Random rnd)
{
    auto r = c.clone;
    static foreach (i; 0 .. speciesGeneCount)
        r.species.tupleof[i] += rate * (speciesHi[i] - speciesLo[i])
            * cast(float) gaussian(rnd);
    foreach (ref seg; r.segments)
        static foreach (i; 0 .. segmentGeneCount)
            seg.tupleof[i] += rate * (segmentHi[i] - segmentLo[i])
                * cast(float) gaussian(rnd);

    if (uniform(0.0f, 1.0f, rnd) < indelRate)
    {
        if (r.segments.length >= maxSegments
            || (r.segments.length > minSegments
                && uniform(size_t(0), 2, rnd) == 0))
        {
            const size_t drop = uniform(size_t(0), r.segments.length, rnd);
            SegmentGenes[] kept;
            foreach (i, seg; r.segments)
                if (i != drop)
                    kept ~= seg;
            r.segments = kept;
        }
        else
            r.segments ~= r.segments[uniform(size_t(0), r.segments.length, rnd)];
    }
    return normalized(r);
}

/// Кроссовер: каждый ген вида достаётся целиком одному из родителей. Фазы
/// обмениваются с учётом разной длины: ребёнок берёт столько фаз, сколько
/// помещается между длинами родителей, каждую — у одного из них.
Chromosome crossover(const Chromosome c, const Chromosome mate, ref Random rnd)
{
    auto r = c.clone;
    static foreach (i; 0 .. speciesGeneCount)
        if (uniform(0.0f, 1.0f, rnd) < 0.5f)
            r.species.tupleof[i] = mate.species.tupleof[i];

    const size_t lo = min(c.segments.length, mate.segments.length);
    const size_t hi = max(c.segments.length, mate.segments.length);
    const size_t len = lo
        + (hi > lo ? uniform(size_t(0), hi - lo + 1, rnd) : 0);

    SegmentGenes[] child;
    foreach (i; 0 .. len)
    {
        const bool hasC = i < c.segments.length;
        const bool hasM = i < mate.segments.length;
        if (hasC && (!hasM || uniform(0.0f, 1.0f, rnd) < 0.5f))
            child ~= c.segments[i];
        else
            child ~= mate.segments[i];
    }
    r.segments = child;
    return normalized(r);
}

/// Аллели — плоский список генов: сначала гены вида, потом все фазы по
/// порядку. Длина списка и есть изменчивость генома.
float[] alleles(const Chromosome c)
{
    float[] r;
    static foreach (i; 0 .. speciesGeneCount)
        r ~= c.species.tupleof[i];
    foreach (seg; c.segments)
        static foreach (i; 0 .. segmentGeneCount)
            r ~= seg.tupleof[i];
    return r;
}

/// Сколько аллелей несёт хромосома: длина `alleles` для сравнения.
size_t alleleCount(const Chromosome c)
{
    return speciesGeneCount + c.segments.length * segmentGeneCount;
}

/// Случайная хромосома вида: равномерно по диапазонам, со случайным числом фаз
/// (если не задано явно). Правило радиусов проверяется сразу.
Chromosome randomChromosome(ref Random rnd, size_t segCount = 0)
{
    Chromosome c;
    static foreach (i; 0 .. speciesGeneCount)
        c.species.tupleof[i] = uniform(speciesLo[i], speciesHi[i], rnd);

    const size_t n = segCount > 0
        ? min(segCount, maxSegments)
        : minSegments
            + uniform(size_t(0), maxSegments - minSegments + 1, rnd);
    c.segments = new SegmentGenes[n];
    foreach (ref seg; c.segments)
        static foreach (i; 0 .. segmentGeneCount)
            seg.tupleof[i] = uniform(segmentLo[i], segmentHi[i], rnd);

    return normalized(c);
}

/// Стандартная нормаль по Боксу–Мюллеру: в Phobos нет `stdNormal`.
private double gaussian(ref Random rnd)
{
    double u1 = uniform(0.0, 1.0, rnd);
    while (u1 == 0.0)
        u1 = uniform(0.0, 1.0, rnd);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * uniform(0.0, 1.0, rnd));
}

/// Каждый ген конечен и лежит в своих границах, а фазы не вырождены.
private void assertGenesOk(const Chromosome g)
{
    static foreach (i; 0 .. speciesGeneCount)
    {
        assert(isFinite(g.species.tupleof[i]), "ген вида не конечен");
        assert(g.species.tupleof[i] >= speciesLo[i] - 1e-6f
            && g.species.tupleof[i] <= speciesHi[i] + 1e-6f,
            "ген вида вне диапазона");
    }
    foreach (seg; g.segments)
    {
        static foreach (i; 0 .. segmentGeneCount)
        {
            assert(isFinite(seg.tupleof[i]), "ген фазы не конечен");
            assert(seg.tupleof[i] >= segmentLo[i] - 1e-6f
                && seg.tupleof[i] <= segmentHi[i] + 1e-6f,
                "ген фазы вне диапазона");
        }
        assert(seg.inhDiffusion >= seg.actDiffusion * inhSpreadFloor - 1e-6f,
            "ингибитор обязан быть шире активатора");
    }
}

unittest
{
    // Хромосома основателя — полный набор генов, все значения конечны:
    // плавающие поля D инициализируются NaN, а молчаливый NaN в гене
    // выглядел бы как «машина не выросла».
    const Chromosome founder;
    assert(founder.segments.length >= minSegments,
        "у основателя есть хотя бы одна фаза");
    assertGenesOk(founder);

    // Имя и индекс — одна и та же память только у вида: он остаётся структурой
    // без общих полей, а фазы живут отдельным списком.
    assert(founder.beamRadius == founder.species.beamRadius);

    // Правило «ингибитор шире активатора» держится на нормализации: даже
    // заведомо вырожденная фаза даёт годную.
    Chromosome degenerate;
    degenerate.segments = [SegmentGenes.init];
    degenerate.segments[0].actDiffusion = 0.8f;
    degenerate.segments[0].inhDiffusion = 0.3f;
    const auto fixed = normalized(degenerate);
    assert(fixed.segments[0].inhDiffusion
        >= fixed.segments[0].actDiffusion * inhSpreadFloor,
        "ингибитор обязан быть шире активатора");
    assert(fixed.segments[0].actDiffusion == 0.8f,
        "нормализация не должна двигать годные гены");
}

unittest
{
    auto rnd = Random(42);
    foreach (_; 0 .. 2000)
    {
        const auto c = randomChromosome(rnd);

        // Каждый ген случайной хромосомы внутри своего диапазона, и правило
        // радиусов выполнено.
        assertGenesOk(c);

        // Мутация и кроссовер не выводят потомка за диапазоны — ни один ген
        // не должен «уехать» в мусор, иначе эволюция слепа.
        const auto mutant = mutated(c, 0.1f, rnd);
        const auto child = crossover(c, randomChromosome(rnd), rnd);
        foreach (g; [mutant, child])
        {
            assertGenesOk(g);
            assert(g.segments.length >= minSegments, "фазы не должны исчезнуть");
            assert(g.segments.length <= maxSegments, "фазы не должны распухнуть");
            assert(alleles(g).length == alleleCount(g),
                "длина аллелей согласована с числом фаз");
        }
    }
}

unittest
{
    // Кроссовер — обмен генами: каждый аллель ребёнка взят у одного из
    // родителей, ничего не смешивается наполовину.
    auto rnd = Random(7);
    Chromosome a;
    a.segments = [SegmentGenes.init];
    a.segments[0].actProduction = 1.0f;
    a.segments[0].inhProduction = 0.25f;
    Chromosome b;
    b.segments = [SegmentGenes.init];
    b.segments[0].actProduction = 3.0f;
    b.segments[0].inhProduction = 0.75f;

    size_t mixed;
    foreach (_; 0 .. 500)
    {
        const auto child = crossover(a, b, rnd);
        const float ap = child.segments[0].actProduction;
        const float ip = child.segments[0].inhProduction;
        assert(ap == 1.0f || ap == 3.0f, "аллель actProduction не из родителей");
        assert(ip == 0.25f || ip == 0.75f, "аллель inhProduction не из родителей");
        if (ap == 3.0f && ip == 0.75f)
            ++mixed;
    }
    assert(mixed > 100, "кроссовер обязан смешивать родителей, а не копировать одного");
}

unittest
{
    // Мутация сильнее слабой: у одного и того же основателя больший шаг
    // уводит признаки дальше. Иначе «темп эволюции» нечем регулировать.
    auto rnd = Random(3);
    const Chromosome founder;
    float nearSum = 0.0f, farSum = 0.0f;
    enum size_t trials = 400;
    foreach (_; 0 .. trials)
    {
        nearSum += abs(founder.segments[0].actProduction
            - mutated(founder, 0.01f, rnd).segments[0].actProduction);
        farSum += abs(founder.segments[0].actProduction
            - mutated(founder, 0.2f, rnd).segments[0].actProduction);
    }
    assert(farSum > nearSum, "шаг мутации должен управлять величиной изменения");
}

unittest
{
    // Индел реально меняет длину списка фаз — иначе «переменная длина» мертва.
    auto rnd = Random(5);
    Chromosome base;
    base.segments = [SegmentGenes.init, SegmentGenes.init, SegmentGenes.init];
    size_t seenShort, seenSame, seenLong;
    foreach (_; 0 .. 500)
    {
        const auto c = mutated(base, 0.02f, rnd);
        if (c.segments.length < base.segments.length) ++seenShort;
        else if (c.segments.length == base.segments.length) ++seenSame;
        else ++seenLong;
    }
    assert(seenShort > 0, "индел обязан уметь удалять фазу");
    assert(seenLong > 0, "индел обязан уметь вставлять фазу");
    assert(seenSame > 0, "индел не обязан срабатывать каждую мутацию");
}
