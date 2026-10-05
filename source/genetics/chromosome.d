module genetics.chromosome;

import std.algorithm : clamp, max;
import std.math : abs, cos, exp, isFinite, log, sqrt, PI;
import std.random : Random, uniform;
import std.traits : FieldNameTuple;

import frame.frame : initialMotorPower;
import physics_world.wheel : defaultWheelRadius;

/// Наименьшее отношение радиуса ингибитора к радиусу активатора. Ниже — система
/// вырождается: ингибитор глушит собственную точку, и рост не идёт дальше
/// первого шага.
enum float inhSpreadFloor = 1.5f;

/// Гены хромосомы: каждое поле — ген, его значение — аллель. Отдельная
/// структура нужна рефлексии: по самому union `FieldNameTuple` сплющивает
/// вложенные поля вместе с `array`, и число генов из него не достать.
struct ChromosomeGenes
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

    /// Сила потока узла (nodal flow): уводит активатор вперёд по курсу и в
    /// одну сторону по ширине, создавая начальную асимметрию — как течение у
    /// ресничек узла смывает молекулы влево. Ноль — рост без оси.
    float flowStrength = -0.15f;

    /// Радиус трубы новой балки, м.
    float beamRadius = 0.045f;

    /// Радиус колеса якоря, м.
    float wheelRadius = defaultWheelRadius;

    /// Сила мотор-колёс, Н·м; знак задаёт направление привода.
    float motorPower = initialMotorPower;

    /// Базовый шаг новой балки, м: дискретизация пространства и её минимальная
    /// длина — без него поля нет, были бы точки, а не отрезки. Фактическая
    /// длина больше и задаётся локальным полем (см. genetics.growth).
    float stepLength = 0.18f;

    /// Скорость роста активатора моторного поля.
    float motorProduction = 0.2f;

    /// Радиус действия активатора моторного поля.
    float motorDiffusion = 2.5f;

    /// Сколько балок организм вообще может нарастить. Размер тела — тоже
    /// свойство вида: иначе поле всегда доращивает каркас до предела, и
    /// отбор лишён выбора между компактной машиной и раздутой.
    float beamBudget = 2.0f;
}

/// Число генов выводится из полей, а не выписывается руками.
enum size_t geneCount = FieldNameTuple!ChromosomeGenes.length;

/**
 * Хромосома — контейнер генов особи. Ген здесь не «балка в точке», а правило
 * роста: каркас вырастает сам из реакционно-диффузионной системы Nodal/Lefty
 * (см. genetics.growth).
 *
 * Union, а не структура: те же гены доступны и по имени, и по индексу в
 * `array`, а это одна и та же память. Имя читается там, где важен конкретный
 * ген; индекс — в общих обходах по всем генам, где имена не нужны.
 *
 * Значения по умолчанию — откалиброванный основатель вида: с ними рост даёт
 * вытянутые ветви с колёсами на концах, то есть нулевое поколение уже едет.
 */
union Chromosome
{
    ChromosomeGenes genes;
    alias this = genes;

    float[geneCount] array;
}

/// Границы аллелей — те же гены в том же порядке, что и `array`. Это свойства
/// вида, а не гены: выход за границу означает «не такая машина», а не другой
/// организм.
enum Chromosome geneLo = {array: [
    0.20f, 0.05f, 0.10f, 0.30f, 0.02f, -0.40f,
    0.02f, 0.10f, -200.0f, 0.08f, 0.02f, 0.10f, 2.00f,
]};

enum Chromosome geneHi = {array: [
    4.00f, 1.00f, 0.80f, 4.00f, 1.50f, 0.40f,
    0.08f, 0.40f, 200.0f, 0.45f, 2.00f, 3.00f, 64.00f,
]};

// Границы обязаны быть заданы у каждого гена и быть конечными: список короче
// geneCount D дополняет NaN, а вырожденный диапазон намертво замораживает ген
// в нормализации. Ловим это здесь, а не в эволюции, потерявшей ген.
static foreach (i; 0 .. geneCount)
{
    static assert(geneHi.array[i] > geneLo.array[i],
        "у гена вырожденный диапазон: не заданы границы");
    static assert(isFinite(geneLo.array[i]) && isFinite(geneHi.array[i]),
        "границы гена не конечны: список короче geneCount");
}

/// Хромосома в допустимых диапазонах: значения зажаты по генам, а ингибитор
/// оставлен шире активатора. Правило держит и мутацию, и кроссовер — иначе
/// особь с негодным соотношением радиусов просто не вырастет.
Chromosome normalized(Chromosome c)
{
    foreach (i; 0 .. geneCount)
        c.array[i] = clamp(c.array[i], geneLo.array[i], geneHi.array[i]);
    c.inhDiffusion = max(c.inhDiffusion, c.actDiffusion * inhSpreadFloor);
    return c;
}

/// Мутация: гауссов шум по каждому гену, шум измеряется долей диапазона гена.
/// Одна мутация меняет все гены разом, но слабо: организм остаётся собой.
Chromosome mutated(Chromosome c, float rate, ref Random rnd)
{
    foreach (i; 0 .. geneCount)
        c.array[i] += rate * (geneHi.array[i] - geneLo.array[i])
            * cast(float) gaussian(rnd);
    return normalized(c);
}

/// Кроссовер: каждый ген достаётся целиком одному из родителей, так что
/// потомок собирает набор признаков из двух особей. Границы и правило
/// «ингибитор шире активатора» проверяются на ребёнке.
Chromosome crossover(Chromosome c, Chromosome mate, ref Random rnd)
{
    foreach (i; 0 .. geneCount)
        if (uniform(0.0f, 1.0f, rnd) < 0.5f)
            c.array[i] = mate.array[i];
    return normalized(c);
}

/// Аллели по генам, для сравнения хромосом целиком.
const(float[geneCount]) alleles(const Chromosome c)
{
    return c.array;
}

/// Случайная хромосома вида: равномерно по диапазонам, правило радиусов
/// проверяется сразу.
Chromosome randomChromosome(ref Random rnd)
{
    Chromosome c;
    foreach (i; 0 .. geneCount)
        c.array[i] = uniform(geneLo.array[i], geneHi.array[i], rnd);
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

/// Каждый ген конечен и лежит в своих границах.
private void assertGenesOk(Chromosome g)
{
    foreach (i; 0 .. geneCount)
    {
        assert(isFinite(g.array[i]), "аллель не конечен");
        assert(g.array[i] >= geneLo.array[i] - 1e-6f
            && g.array[i] <= geneHi.array[i] + 1e-6f, "аллель вне диапазона");
    }
}

unittest
{
    // Хромосома основателя — полный набор генов, все значения конечны:
    // плавающие поля D инициализируются NaN, а молчаливый NaN в гене
    // выглядел бы как «машина не выросла».
    const Chromosome founder;
    assertGenesOk(founder);

    // Имя и индекс — одна и та же память, а не две копии.
    static assert(Chromosome.actProduction.offsetof == Chromosome.array.offsetof);
    assert(founder.actProduction == founder.array[0]);

    // Правило «ингибитор шире активатора» держится на нормализации: даже
    // заведомо вырожденная хромосома даёт годную.
    Chromosome degenerate;
    degenerate.actDiffusion = 0.8f;
    degenerate.inhDiffusion = 0.3f;
    const auto fixed = normalized(degenerate);
    assert(fixed.inhDiffusion >= fixed.actDiffusion * inhSpreadFloor,
        "ингибитор обязан быть шире активатора");
    assert(fixed.actDiffusion == 0.8f, "нормализация не должна двигать годные гены");
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
        assert(c.inhDiffusion >= c.actDiffusion * inhSpreadFloor,
            "случайная хромосома нарушает правило радиусов");

        // Мутация и кроссовер не выводят потомка за диапазоны — ни один ген
        // не должен «уехать» в мусор, иначе эволюция слепа.
        const auto mutant = mutated(c, 0.1f, rnd);
        const auto child = crossover(c, randomChromosome(rnd), rnd);
        foreach (g; [mutant, child])
            assertGenesOk(g);
        assert(mutant.inhDiffusion >= mutant.actDiffusion * inhSpreadFloor);
        assert(child.inhDiffusion >= child.actDiffusion * inhSpreadFloor);
    }
}

unittest
{
    // Кроссовер — обмен генами: каждый аллель ребёнка взят у одного из
    // родителей, ничего не смешивается наполовину.
    auto rnd = Random(7);
    Chromosome a;
    a.actProduction = 1.0f;
    a.inhProduction = 0.25f;
    Chromosome b;
    b.actProduction = 3.0f;
    b.inhProduction = 0.75f;

    size_t mixed;
    foreach (_; 0 .. 500)
    {
        const auto child = crossover(a, b, rnd);
        const float ap = child.actProduction;
        const float ip = child.inhProduction;
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
    float nearSum, farSum;
    enum size_t trials = 400;
    foreach (_; 0 .. trials)
    {
        nearSum += abs(founder.actProduction
            - mutated(founder, 0.01f, rnd).actProduction);
        farSum += abs(founder.actProduction
            - mutated(founder, 0.2f, rnd).actProduction);
    }
    assert(farSum > nearSum, "шаг мутации должен управлять величиной изменения");
}
