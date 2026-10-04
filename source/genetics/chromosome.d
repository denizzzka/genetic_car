module genetics.chromosome;

import std.algorithm : clamp, max;
import std.math : abs, cos, exp, isFinite, log, sqrt, PI;
import std.random : Random, uniform;

import frame.frame : initialMotorPower;
import physics_world.wheel : defaultWheelRadius;

/// Число генов хромосомы.
enum size_t geneCount = 10;

/// Границы аллелей, в порядке полей `Chromosome`. Это свойства вида, а не
/// гены: выход за границу означает «не такая машина», а не другой организм.
enum float[geneCount] geneLo = [
    0.20f, 0.05f, 0.10f, 0.30f, 0.02f, -0.40f,
    0.02f, 0.10f, -200.0f, 0.08f,
];
enum float[geneCount] geneHi = [
    4.00f, 1.00f, 0.80f, 4.00f, 1.50f, 0.40f,
    0.08f, 0.40f, 200.0f, 0.45f,
];

/// Наименьшее отношение радиуса ингибитора к радиусу активатора. Ниже — система
/// вырождается: ингибитор глушит собственную точку, и рост не идёт дальше
/// первого шага.
enum float inhSpreadFloor = 1.5f;

/**
 * Хромосома — контейнер генов особи, каждое поле — ген, его значение —
 * аллель. Ген здесь не «балка в точке», а правило роста: каркас вырастает
 * сам из реакционно-диффузионной системы Nodal/Lefty (см. genetics.growth).
 *
 * Значения по умолчанию — откалиброванный основатель вида: с ними рост даёт
 * вытянутые ветви с колёсами на концах, то есть нулевое поколение уже едет.
 */
struct Chromosome
{
    /// Скорость роста активатора: насколько быстро активатор размножает себя
    /// там, где он уже есть. Мало — рост вялый, много — форма плывёт.
    float actProduction = 1.5f;

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
    float threshold = 0.35f;

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

    /// Аллели по генам в порядке полей `Chromosome`.

    const(float[geneCount]) alleles() const
{
    return [actProduction, inhProduction, actDiffusion, inhDiffusion, threshold,
        flowStrength, beamRadius, wheelRadius, motorPower, stepLength];
}

    /// Хромосома в допустимых диапазонах: значения зажаты по генам, а ингибитор
    /// оставлен шире активатора. Правило держит и мутацию, и кроссовер — иначе
    /// особь с негодным соотношением радиусов просто не вырастет.
    Chromosome normalized() const
    {
        float[geneCount] a = alleles();
        foreach (i; 0 .. geneCount)
            a[i] = clamp(a[i], geneLo[i], geneHi[i]);
        a[3] = max(a[3], a[2] * inhSpreadFloor);
        return ofAlleles(a);
    }

    /// Мутация: гауссов шум по каждому гену, шум измеряется долей диапазона гена.
    /// Одна мутация меняет все гены разом, но слабо: организм остаётся собой.
    Chromosome mutated(float rate, ref Random rnd) const
    {
        float[geneCount] a = alleles();
        foreach (i; 0 .. geneCount)
            a[i] += rate * (geneHi[i] - geneLo[i]) * cast(float) gaussian(rnd);
        return ofAlleles(a).normalized();
    }

    /// Кроссовер: каждый ген достаётся целиком одному из родителей, так что
    /// потомок собирает набор признаков из двух особей. Границы и правило
    /// «ингибитор шире активатора» проверяются на ребёнке.
    Chromosome crossover(Chromosome mate, ref Random rnd) const
    {
        float[geneCount] a = alleles();
        const auto b = mate.alleles;
        foreach (i; 0 .. geneCount)
            if (uniform(0.0f, 1.0f, rnd) < 0.5f)
                a[i] = b[i];
        return ofAlleles(a).normalized();
    }
}

/// Хромосома из набора аллелей (порядок — как в `alleles`).
Chromosome ofAlleles(const float[geneCount] a)
{
    Chromosome c;
    c.actProduction = a[0];
    c.inhProduction = a[1];
    c.actDiffusion = a[2];
    c.inhDiffusion = a[3];
    c.threshold = a[4];
    c.flowStrength = a[5];
    c.beamRadius = a[6];
    c.wheelRadius = a[7];
    c.motorPower = a[8];
    c.stepLength = a[9];
    return c;
}

/// Случайная хромосома вида: равномерно по диапазонам, правило радиусов
/// проверяется сразу.
Chromosome randomChromosome(ref Random rnd)
{
    float[geneCount] a;
    foreach (i; 0 .. geneCount)
        a[i] = uniform(geneLo[i], geneHi[i], rnd);
    return ofAlleles(a).normalized();
}

/// Стандартная нормаль по Боксу–Мюллеру: в Phobos нет `stdNormal`.
private double gaussian(ref Random rnd)
{
    double u1 = uniform(0.0, 1.0, rnd);
    while (u1 == 0.0)
        u1 = uniform(0.0, 1.0, rnd);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * uniform(0.0, 1.0, rnd));
}

unittest
{
    // Хромосома основателя — полный набор генов, все значения конечны:
    // плавающие поля D инициализируются NaN, а молчаливый NaN в гене
    // выглядел бы как «машина не выросла».
    const Chromosome founder;
    foreach (i, v; founder.alleles)
    {
        assert(isFinite(v), "аллель не конечен");
        assert(v >= geneLo[i] && v <= geneHi[i], "аллель вне диапазона");
    }

    // Правило «ингибитор шире активатора» держится на нормализации: даже
    // заведомо вырожденная хромосома даёт годную.
    Chromosome degenerate;
    degenerate.actDiffusion = 0.8f;
    degenerate.inhDiffusion = 0.3f;
    const auto fixed = degenerate.normalized;
    assert(fixed.inhDiffusion >= fixed.actDiffusion * inhSpreadFloor,
        "ингибитор обязан быть шире активатора");
    assert(fixed.actDiffusion == 0.8f, "нормализация не должна двигать годные гены");
}

unittest
{
    import std.random : Random;

    auto rnd = Random(42);
    foreach (_; 0 .. 2000)
    {
        const auto c = randomChromosome(rnd);

        // Каждый ген случайной хромосомы внутри своего диапазона, и правило
        // радиусов выполнено.
        foreach (i, v; c.alleles)
            assert(v >= geneLo[i] - 1e-6f && v <= geneHi[i] + 1e-6f,
                "случайный аллель вне диапазона");
        assert(c.inhDiffusion >= c.actDiffusion * inhSpreadFloor,
            "случайная хромосома нарушает правило радиусов");

        // Мутация и кроссовер не выводят потомка за диапазоны — ни один ген
        // не должен «уехать» в мусор, иначе эволюция слепа.
        const auto mutant = c.mutated(0.1f, rnd);
        const auto child = c.crossover(randomChromosome(rnd), rnd);
        foreach (g; [mutant, child])
            foreach (i, v; g.alleles)
            {
                assert(v >= geneLo[i] - 1e-6f && v <= geneHi[i] + 1e-6f,
                    "потомок вне диапазона");
                assert(v == v, "потомок с NaN в гене");
            }
        assert(mutant.inhDiffusion >= mutant.actDiffusion * inhSpreadFloor);
        assert(child.inhDiffusion >= child.actDiffusion * inhSpreadFloor);
    }
}

unittest
{
    import std.random : Random;

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
        const auto child = a.crossover(b, rnd);
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
    import std.random : Random;

    // Мутация сильнее слабой: у одного и того же основателя больший шаг
    // уводит признаки дальше. Иначе «темп эволюции» нечем регулировать.
    auto rnd = Random(3);
    const Chromosome founder;
    float nearSum, farSum;
    enum size_t trials = 400;
    foreach (_; 0 .. trials)
    {
        nearSum += abs(founder.actProduction
            - founder.mutated(0.01f, rnd).actProduction);
        farSum += abs(founder.actProduction
            - founder.mutated(0.2f, rnd).actProduction);
    }
    assert(farSum > nearSum, "шаг мутации должен управлять величиной изменения");
}