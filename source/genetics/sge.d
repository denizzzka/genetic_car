module genetics.sge;

import std.array : appender;
import std.random;
import std.sumtype;

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

    this()
    {
    }

    this(size_t nGenes)
    {
        genes.length = nGenes;
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
    return child;
}

void mutate(Genotype genotype, float probability, ref Random rnd)
{
    foreach (ref gene; genotype.genes)
        foreach (ref codon; gene)
            if (uniform(0.0f, 1.0f, rnd) < probability)
                codon = uniform(0u, uint.max, rnd);
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
 */
Terminal!TokT[] decode(TokT)(const Grammar gr, const Genotype genotype, out bool ok)
{
    ok = false;

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

    ok = true;
    return result.data;
}
