module genetics.sge;

import std.algorithm.comparison : max, min;
import std.array: appender, insertInPlace;
import std.math : cos, exp, log, PI, round, sqrt;
import std.random;
import std.sumtype;

/// Стартовые темпы мутаций поколения 0; дальше особи самоадаптируются.
enum size_t defaultPointHits = 3;
enum size_t defaultIndelHits = 1;
enum float defaultStructuralChance = 0.25f;

abstract class Symbol {}

final class Terminal(TokT) : Symbol
{
    TokT tok;
    SumType!(int, float) payload;

    this(TokT tok)
    {
        this.tok = tok;
    }

    this(TokT tok, int i)
    {
        this(tok);
        payload = i;
    }

    this(TokT tok, float f)
    {
        this(tok);
        payload = f;
    }

    auto f() const => payload.tryGet!(const float);
    auto i() const => payload.tryGet!(const int);
}

class NonTerminal : Symbol
{
    immutable string name;
    size_t id;
    Production[] productions;

    this(string name)
    {
        this.name = name;
    }
}

/**
 * Самплер — нетерминал-лист с собственным геном: при раскрытии берёт
 * следующий кодон своего гена и превращает его в значение токена.
 *
 * float: `min + (codon / uint.max) * (max - min)`.
 * int: `codon % bound`.
 */
final class Sampler(TokT) : NonTerminal
{
    TokT tok;
    float min;
    float max;
    bool isInt;
    size_t bound;

    this(string name, TokT tok, float min, float max)
    {
        super(name);
        this.tok = tok;
        this.min = min;
        this.max = max;
    }

    this(string name, TokT tok, size_t bound)
    {
        super(name);
        this.tok = tok;
        this.bound = bound;
        isInt = true;
    }

    Terminal!TokT sample(TokT tok, uint codon) const
    {
        if (isInt)
            return new Terminal!TokT(tok, cast(int)(codon % bound));
        else
        {
            const t = min + (cast(float)codon / cast(float)uint.max) * (max - min);
            return new Terminal!TokT(tok, t);
        }
    }
}

final class Production
{
    Symbol[] symbols;

    this(Symbol[] symbols)
    {
        this.symbols = symbols;
    }
}

final class Grammar
{
    NonTerminal start;
    NonTerminal[] symbols;

    this(NonTerminal start, NonTerminal[] symbols)
    {
        this.start = start;
        this.symbols = symbols;
        foreach (i, s; this.symbols)
            s.id = i;
    }
}

/// Генотип SGE: один ген (список кодонов) на каждый нетерминал грамматики.
final class Genotype
{
    uint[][] genes;

    /// Самоадаптируемые темпы мутаций потомков особи.
    size_t pointHits = defaultPointHits;
    size_t indelHits = defaultIndelHits;
    float structuralChance = defaultStructuralChance;

    this()
    {
    }

    this(size_t nGenes)
    {
        genes.length = nGenes;
    }

    /// Глубокая копия генома
    Genotype dup() const
    {
        auto copy = new Genotype(this.genes.length);

        foreach (g, ref gene; this.genes)
            copy.genes[g] = gene.dup;

        copy.pointHits = pointHits;
        copy.indelHits = indelHits;
        copy.structuralChance = structuralChance;

        return copy;
    }
}

version (unittest)
{
    /// Случайный геном: каждый ген — список случайных кодонов.
    Genotype randomGenotype(const Grammar gr, size_t maxGeneLength, ref Random rnd)
    {
        auto genotype = new Genotype(gr.symbols.length);
        foreach (ref gene; genotype.genes)
        {
            gene.length = uniform(cast(size_t)1, maxGeneLength + 1, rnd);
            foreach (ref codon; gene)
                codon = uniform(0u, uint.max, rnd);
        }
        return genotype;
    }
}

Genotype crossover(const Genotype a, const Genotype b, ref Random rnd)
{
    auto child = new Genotype(a.genes.length);
    foreach (g; 0 .. a.genes.length)
    {
        if (uniform(0.0f, 1.0f, rnd) < 0.5f)
            child.genes[g] = a.genes[g].dup;
        else
            child.genes[g] = b.genes[g].dup;
    }
    child.pointHits = a.pointHits;
    child.indelHits = a.indelHits;
    child.structuralChance = a.structuralChance;
    return child;
}

/**
 * Мутирует геном: переворачивает ровно `hits` случайно выбранных кодонов.
 *
 * Вероятностная мутация каждого кодона при сотнях кодонов в геноме давала
 * бы десятки правок за нажатие и каскадно разрушала структуру: ген
 * `beamList` кодирует длину цепочки балок, и один флип "продолжить"->"конец"
 * обрывает почти всю раму. Фиксированное число правок делает одну мутацию
 * умеренным изменением фенотипа.
 *
 * `weights` — вес каждого гена при выборе мишени правки: ген-токсикант
 * (индексы узлов) получает нуль, полезные (геометрия) — единицу, тогда
 * правки не тратятся впустую. Без весов выбор — равномерный по всем кодонам.
 */
void mutate(Genotype genotype, size_t hits, ref Random rnd,
    const float[] weights = null)
{
    if (hits == 0)
        return;

    const bool weighted = weights !is null && weights.length == genotype.genes.length;
    if (weighted)
    {
        foreach (_; 0 .. hits)
        {
            auto gi = weightedGeneIndex(weights, rnd);
            auto gene = genotype.genes[gi];
            if (gene.length == 0)
                continue;
            gene[uniform(0, gene.length, rnd)] = uniform(0u, uint.max, rnd);
        }
        return;
    }

    size_t total;
    foreach (gene; genotype.genes)
        total += gene.length;
    if (total == 0)
        return;

    foreach (_; 0 .. hits)
    {
        auto pos = uniform(0, total, rnd);
        foreach (ref gene; genotype.genes)
        {
            if (pos < gene.length)
            {
                gene[pos] = uniform(0u, uint.max, rnd);
                break;
            }
            pos -= gene.length;
        }
    }
}

/// Индекс гена по его весу: правки целятся в малочисленные полезные гены.
private size_t weightedGeneIndex(const float[] w, ref Random rnd)
{
    float total = 0.0f;
    foreach (x; w)
        total += x;
    if (total <= 0.0f)
        return uniform(0, w.length, rnd);

    float v = uniform(0.0f, total, rnd);
    foreach (i, x; w)
    {
        if (v < x)
            return i;
        v -= x;
    }
    return w.length - 1;
}

/**
 * Индельная мутация: вставляет или удаляет ровно `hits` кодонов.
 *
 * Единственный оператор, меняющий длины генов, а значит и потолок
 * рекурсивных списков (`beamList`, `anchorList`): сколько кодонов в гене —
 * столько максимум элементов. Нужен для правил развития (зеркалирование,
 * ветвление, повтор N раз, градиент), где число повторений и параметров
 * должно уметь расти.
 *
 * Ген никогда не удаляется целиком: если удалять нечего, вставка выполняется
 * принудительно. Пустой ген обнулил бы `decode` использующего его символа.
 */
void mutateIndel(Genotype genotype, size_t hits, ref Random rnd)
{
    if (hits == 0)
        return;

    foreach (_; 0 .. hits)
    {
        size_t total;
        foreach (gene; genotype.genes)
            total += gene.length;
        if (total == 0)
            return;

        auto pos = uniform(0, total, rnd);
        size_t gi;
        while (pos >= genotype.genes[gi].length)
        {
            pos -= genotype.genes[gi].length;
            ++gi;
        }

        const canDelete = genotype.genes[gi].length > 1;
        const insert = !canDelete || uniform(0, 2, rnd) == 0;

        if (insert)
            genotype.genes[gi].insertInPlace(pos, uniform(0u, uint.max, rnd));
        else
        {
            auto gene = genotype.genes[gi];
            genotype.genes[gi] = gene[0 .. pos] ~ gene[pos + 1 .. $];
        }
    }
}

/**
 * Самоадаптация темпов мутаций: наследник получает параметры родителя с
 * лог-нормальным шагом по числу правок и гауссовым шагом по доле структурных
 * мутаций. Темп эволюции становится признаком, а не константой конфигурации.
 */
void mutateSelfAdaptation(Genotype genotype, ref Random rnd)
{
    genotype.pointHits = evolveCount(genotype.pointHits, 1, 12, rnd);
    genotype.indelHits = evolveCount(genotype.indelHits, 1, 6, rnd);
    genotype.structuralChance = cast(float)
        max(0.0, min(1.0, cast(double) genotype.structuralChance
            + 0.1 * gaussian(rnd)));
}

/// Лог-нормальный шаг числа правок с зажимом в разрешённый диапазон.
private size_t evolveCount(size_t v, size_t lo, size_t hi, ref Random rnd)
{
    const double raw = round((cast(double) v) * exp(0.3 * gaussian(rnd)));
    const ulong clamped = max(cast(ulong) lo, min(cast(ulong) hi, cast(ulong) raw));
    return cast(size_t) clamped;
}

/// Стандартная нормаль по Боксу–Мюллеру: в Phobos нет `stdNormal`.
private double gaussian(ref Random rnd)
{
    double u1 = uniform(0.0, 1.0, rnd);
    while (u1 == 0.0)
        u1 = uniform(0.0, 1.0, rnd);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * uniform(0.0, 1.0, rnd));
}

/**
 * Расшифровка генома в последовательность терминалов.
 *
 * Обход дерева вывода в глубину: левосторонний разворот продукции, поэтому
 * терминалы выходят в том же порядке, что и в грамматике. Раскрытый
 * нетерминал берёт следующий кодон из своего гена; при исчерпании гена
 * кодоны переиспользуются по кругу. Интроны — гены символов, не
 * встречающихся в данном дереве, — не затрагиваются и передаются потомкам
 * как есть.
 *
 * Возвращает null, если декодирование невозможно (пустой ген, слишком
 * много развёрток); сам результат выступает признаком успеха.
 */
Terminal!TokT[] decode(TokT)(const Grammar gr, const Genotype genotype)
{
    Symbol[] stack;
    stack ~= cast(NonTerminal) gr.start;
    size_t expansions = 0;
    enum size_t maxExpansions = 10000;
    size_t[] used = new size_t[gr.symbols.length];

    auto result = appender!(Terminal!TokT[])();
    while (stack.length > 0 && expansions < maxExpansions)
    {
        auto sym = stack[$ - 1];
        stack.length -= 1;
        auto sp = cast(Sampler!TokT) sym;
        if (sp !is null)
        {
            auto gene = genotype.genes[sp.id];
            if (gene.length == 0)
                return null;
            auto codon = gene[used[sp.id] % gene.length];
            ++used[sp.id];
            result.put(sp.sample(sp.tok, codon));
            continue;
        }

        auto nt = cast(NonTerminal) sym;
        if (nt !is null)
        {
            auto gene = genotype.genes[nt.id];
            if (gene.length == 0)
                return null;
            auto codon = gene[used[nt.id] % gene.length];
            ++used[nt.id];
            auto production = nt.productions[codon % nt.productions.length];

            foreach_reverse (s; production.symbols)
                stack ~= s;
            ++expansions;
            continue;
        }

        result.put(cast(Terminal!TokT) sym);
    }

    if (expansions >= maxExpansions)
        return null;

    return result.data;
}

unittest
{
    import std.random : Random;

    auto g = new Genotype(3);
    g.genes = [[5u, 5u], [5u, 5u, 5u], [5u, 5u, 5u, 5u]];
    auto g0 = new Genotype(3);
    foreach (i, ref gene; g.genes)
        g0.genes[i] = gene.dup;

    auto rnd = Random(1);
    mutate(g, 4, rnd);

    size_t diffs;
    foreach (i, ref gene; g.genes)
        foreach (j, codon; gene)
            if (codon != g0.genes[i][j])
                ++diffs;
    assert(diffs == 4, "mutate должен менять ровно hits кодонов");

    mutate(g, 0, rnd);
    size_t diffs0;
    foreach (i, ref gene; g.genes)
        foreach (j, codon; gene)
            if (codon != g0.genes[i][j])
                ++diffs0;
    assert(diffs0 == 4, "mutate с hits == 0 ничего не меняет");
}

unittest
{
    import std.random : Random;

    // Взвешенная мутация: ген с нулевым весом не трогается, все правки
    // ложатся только в гены с ненулевым весом.
    auto g = new Genotype(3);
    g.genes = [[111u, 222u], [333u, 444u], [555u, 666u]];
    const weights = [0.0f, 0.0f, 1.0f];
    auto rnd = Random(4);
    mutate(g, 40, rnd, weights);
    assert(g.genes[0] == [111u, 222u] && g.genes[1] == [333u, 444u],
        "нулевой вес исключает ген из мишеней");
    assert(g.genes[2] != [555u, 666u],
        "все правки легли в ген с ненулевым весом");
}

unittest
{
    import std.random: Random;

    // Ген из одного кодона удалить нельзя — единственная правка станет вставкой.
    auto g = new Genotype(1);
    g.genes = [[7u]];
    auto rnd = Random(1);
    mutateIndel(g, 1, rnd);
    assert(g.genes[0].length == 2, "ген не должен становиться пустым");

    // Ни один ген не удаляется целиком даже при большом числе правок,
    // а суммарная длина меняется не сильнее, чем на hits.
    auto g2 = new Genotype(3);
    g2.genes = [[1u, 2u, 3u], [4u], [5u, 6u]];
    auto rnd2 = Random(2);
    mutateIndel(g2, 50, rnd2);
    size_t total;
    foreach (gene; g2.genes)
    {
        assert(gene.length >= 1, "ген не должен стать пустым");
        total += gene.length;
    }
    assert(total >= 3 && total <= 56, "длина генома не должна уезжать на произвол");

    // hits == 0 ничего не меняет.
    auto g3 = g2.dup;
    mutateIndel(g3, 0, rnd2);
    assert(g3.genes == g2.genes);
}

unittest
{
    import std.random : Random;

    // dup  и crossover переносят темпы мутаций особи.
    auto a = new Genotype(2);
    a.pointHits = 5;
    a.indelHits = 2;
    a.structuralChance = 0.7f;
    auto d = a.dup;
    assert(d.pointHits == 5 && d.indelHits == 2 && d.structuralChance == 0.7f,
        "dup сохраняет самоадаптируемые темпы");

    auto b = new Genotype(2);
    auto rnd = Random(1);
    auto child = crossover(a, b, rnd);
    assert(child.pointHits == 5 && child.indelHits == 2 && child.structuralChance == 0.7f,
        "crossover наследует темпы от основного родителя");
}

unittest
{
    import std.random : Random;

    // Шаг самоадаптации не разряжается в опасные диапазоны и не застревает.
    auto rnd = Random(2);
    foreach (_; 0 .. 10000)
    {
        auto g = new Genotype(1);
        g.genes = [[0u]];
        mutateSelfAdaptation(g, rnd);
        assert(g.pointHits >= 1 && g.pointHits <= 12, "pointHits вне диапазона");
        assert(g.indelHits >= 1 && g.indelHits <= 6, "indelHits вне диапазона");
        assert(g.structuralChance >= 0.0f && g.structuralChance <= 1.0f,
            "structuralChance вне [0,1]");
    }
}
