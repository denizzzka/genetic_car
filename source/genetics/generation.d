module genetics.generation;

import std.algorithm : sort, map, min, max;
import std.array : array;
import std.exception : enforce;
import std.random;
import std.typecons : Nullable;

import genetics.buggygrammar;
import genetics.fitness;
import genetics.selection;
import genetics.sge;

private enum float elitePercent = 5.0f;

/// Попыток мутации одного элемента, прежде чем тот считается провальным.
private enum size_t maxMutationAttempts = 30;

/// Полных проходов по массиву родителей вне элиты, после которых
/// незаполненные слоты поколения — повод для `enforce`.
private enum size_t maxParentCycles = 3;

/// Жизнеспособный мутант `base` для слотов будущей физической симуляции.
/// Темпы мутаций наследуются с самоадаптацией; число правок — из параметров
/// особи, а доля структурных мутаций перебрасывается на каждой попытке.
Nullable!Genotype tryCreateViableMutant(const Grammar gr, const Genotype base,
    ref Random rnd)
{
    auto candidate = base.dup;
    mutateSelfAdaptation(candidate, rnd);
    const weights = mutationWeights(gr);

    foreach (_; 0 .. maxMutationAttempts)
    {
        auto trial = candidate.dup;
        const structural = uniform(0.0f, 1.0f, rnd) < candidate.structuralChance;
        if (structural)
            mutateIndel(trial, candidate.indelHits, rnd);
        else
            mutate(trial, candidate.pointHits, rnd, weights);

        auto dev = develop(gr, trial);
        if (!dev.isNull && buggyFitness(dev.get.frame, dev.get.ast) > 0.0f)
            return Nullable!Genotype(trial);
    }
    return Nullable!Genotype.init;
}

/// Построение следующего поколения — заполнение слотов физической симуляции.
///
/// Слоты заполняются, пока хватает бюджета `maxParentCycles` циклов по
/// родителям вне элиты; когда бюджет исчерпан, а слоты не заполнены — `enforce`.
Genotype[] buildNextGeneration(const Grammar gr, Individual[] pop,
    const EvolutionConfig p, ref Random rnd)
{
    assert(pop.length > 0,
        "не из чего строить следующее поколение: родителей нет");

    auto ranked = pop.dup.sort!((a, b) => a.fitness > b.fitness).release;

    Genotype[] next;
    next.reserve(EvolutionConfig.populationSize);

    // Элита: лучшие elitePercent% родителей переходят без изменений.
    const elites = min(EvolutionConfig.populationSize,
        max(cast(size_t) 1,
            cast(size_t) (ranked.length * elitePercent / 100.0f)));
    next ~= ranked[0 .. elites].map!(e => e.genotype.dup).array;

    // Заполнение остальных слотов: диапазон родителей вне элиты (лучшие
    // первыми), по нему счётчик идёт по кругу; на каждого — кроссовер со
    // случайным партнёром (турнир по всей популяции) и до maxMutationAttempts
    // мутаций получившегося ребёнка.
    auto pool = ranked[elites .. $];
    size_t parentIdx;
    for (size_t visited; next.length < EvolutionConfig.populationSize; ++visited)
    {
        enforce(visited < maxParentCycles * pool.length,
            "не удалось заполнить кандидаты физической симуляции: "
            ~ "исчерпаны 3 цикла по родителям вне элиты");

        const base = pool[parentIdx].genotype;
        const mate = ranked[tournament(ranked, p.tournamentSize, rnd)].genotype;
        // Кроссовер смешивает геном «главного» родителя (по очереди из пула)
        // и случайного партнёра: потомок наследует куски обоих. Мутация и
        // проверка жизнеспособности этого потомка — дальше, в tryCreateViableMutant.
        if (auto candidate = tryCreateViableMutant(gr,
            crossover(base, mate, rnd), rnd))
            next ~= candidate.get;
        parentIdx = (parentIdx + 1) % pool.length;
    }
    return next;
}

unittest
{
    import std.random : Random;

    auto gr = buggyGrammar();
    auto rnd = Random(5);
    auto pop = evaluatePopulation(gr, seedPopulation(gr, EvolutionConfig.populationSize));

    // tryCreateViableMutant: из заведомо жизнеспособного родителя живой мутант
    // находится практически всегда; non-Null результат обязан стать
    // жизнеспособным каркасом, Null — разрешённый провал.
    bool found;
    foreach (e; pop)
    {
        auto may = tryCreateViableMutant(gr, e.genotype, rnd);
        if (!may.isNull)
        {
            auto dev = develop(gr, may.get);
            assert(!dev.isNull && buggyFitness(dev.get.frame, dev.get.ast) > 0.0f,
                "не-Null результат обязан стать жизнеспособным каркасом");
            found = true;
        }
    }
    assert(found, "на множестве базовых родителей должен найтись живой мутант");

    // buildNextGeneration: все слоты заполнены, каждый статически жив,
    // кроссовер и мутации не портят размер генома.
    EvolutionConfig cfg;
    auto next = buildNextGeneration(gr, pop, cfg, rnd);
    assert(next.length == EvolutionConfig.populationSize, "поколение целиком заполнено");
    foreach (g; next)
    {
        auto dev = develop(gr, g);
        assert(!dev.isNull && buggyFitness(dev.get.frame, dev.get.ast) > 0.0f,
            "каждая особь поколения статически жизнеспособна");
        assert(g.genes.length == gr.symbols.length,
            "кроссовер и мутации сохраняют число генов");
    }

    // Элита: лучший родитель переходит в следующее поколение без изменений.
    Individual[] ranked = new Individual[pop.length];
    foreach (i, e; pop)
        ranked[i] = e;
    sort!((a, b) => a.fitness > b.fitness)(ranked);
    assert(next[0].genes == ranked[0].genotype.genes,
        "лучший родитель сохраняется в следующее поколение неизменным");
}