module genetics.sge;

import std.array : appender;
import std.random;
import std.sumtype;

abstract class Symbol {}

final class Terminal(TokT) : Symbol
{
    TokT tok;
    SumType!(int, float) payload;

    private this(TokT tok)
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

final class NonTerminal : Symbol
{
    immutable string name;
    size_t id;
    Production[] productions;

    this(string name)
    {
        this.name = name;
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
 * BFS по дереву вывода: раскрытый нетерминал берёт следующий кодон из своего
 * гена; при исчерпании гена кодоны переиспользуются по кругу. Интроны —
 * гены символов, не встречающихся в данном дереве, — не затрагиваются и
 * передаются потомкам как есть.
 */
Terminal!TokT[] decode(TokT)(const Grammar gr, const Genotype genotype, out bool ok)
{
    ok = false;

    Symbol[] queue;
    queue ~= cast(NonTerminal) gr.start;
    size_t head = 0;
    size_t expansions = 0;
    enum size_t maxExpansions = 1000;
    size_t[] used = new size_t[gr.symbols.length];

    auto result = appender!(Terminal!TokT[])();
    while (head < queue.length && expansions < maxExpansions)
    {
        auto sym = queue[head++];
        auto nt = cast(NonTerminal) sym;
        if (nt is null)
        {
            result.put(cast(Terminal!TokT) sym);
            continue;
        }

        auto gene = genotype.genes[nt.id];
        if (gene.length == 0)
            return null;
        auto codon = gene[used[nt.id] % gene.length];
        ++used[nt.id];
        auto production = nt.productions[codon % nt.productions.length];
        queue ~= production.symbols;
        ++expansions;
    }

    if (expansions >= maxExpansions)
        return null;

    ok = true;
    return result.data;
}
