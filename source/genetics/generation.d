module genetics.generation;

import std.algorithm : sort, map, min, max;
import std.array : array;
import std.exception : enforce;
import std.random;
import std.typecons : Nullable;

import genetics.chromosome : Chromosome, geneCount;
import genetics.fitness : buggyFitness;
import genetics.growth : develop;
import genetics.selection;

private enum float elitePercent = 5.0f;

/// Попыток мутации одного ребёнка, прежде чем он считается провальным.
private enum size_t maxMutationAttempts = 30;

/// Полных проходов по массиву родителей, после которых
/// незаполненные слоты поколения — повод для `enforce`.
private enum size_t maxParentCycles = 3;

/// Вырос ли каркас и поехал ли: без этого гейта в физику отправляется пустота.
private bool viable(const Chromosome chr)
{
    if (auto grown = develop(chr))
        return buggyFitness(grown.get) > 0.0f;
    return false;
}

/// Сколько колёс вырасти удалось — мера того, насколько ребёнок близок к
/// годной машине, когда годной машины не вышло вовсе.
private size_t anchorCount(const Chromosome chr)
{
    if (auto grown = develop(chr))
        return grown.get.anchors.length;
    return 0;
}

/**
 * Жизнеспособный мутант `base` для слотов будущей физической симуляции.
 *
 * Мутация всегда берётся от `base`, а не от предыдущей пробы: накапливающийся
 * дрейф уводит цепочку всё дальше от родителя, и после десятка проб ни одна
 * машина уже не вырастает. Проб до `maxMutationAttempts` достаточно: при
 * заметной доле выживающих родитель всегда даёт годного ребёнка.
 *
 * Если годного ребёнка нет всё же нет, возвращается лучшее из виденного —
 * родитель или самая «колёсистая» проба. Отбор сам отбракует такой каркас по
 * нулевому фитнесу, но эволюция не встанет колом.
 */
Chromosome tryCreateViableMutant(const Chromosome base, const EvolutionConfig p,
    ref Random rnd)
{
    Chromosome best = base;
    size_t bestAnchors = anchorCount(base);
    foreach (_; 0 .. maxMutationAttempts)
    {
        const auto trial = base.mutated(p.mutationRate, rnd);
        if (viable(trial))
            return trial;
        const size_t anchors = anchorCount(trial);
        if (anchors > bestAnchors)
        {
            best = trial;
            bestAnchors = anchors;
        }
    }
    return best;
}

/// Построение следующего поколения — заполнение слотов физической симуляции.
///
/// Слоты заполняются, пока хватает бюджета `maxParentCycles` циклов по
/// родителям; когда бюджет исчерпан, а слоты не заполнены — `enforce`.
Chromosome[] buildNextGeneration(Individual[] pop, const EvolutionConfig p,
    ref Random rnd)
{
    assert(pop.length > 0,
        "не из чего строить следующее поколение: родителей нет");

    auto ranked = pop.dup.sort!((a, b) => a.fitness > b.fitness).release;

    Chromosome[] next;
    next.reserve(EvolutionConfig.populationSize);

    // Элита: лучшие elitePercent% родителей переходят без изменений.
    const elites = min(EvolutionConfig.populationSize,
        max(cast(size_t) 1,
            cast(size_t) (ranked.length * elitePercent / 100.0f)));
    next ~= ranked[0 .. elites].map!(e => e.chromosome).array;

    // Заполнение остальных слотов: родители — все, лучшие первыми, по ним
    // счётчик идёт по кругу; на каждого — кроссовер со случайным партнёром
    // (турнир по всей популяции) и до maxMutationAttempts мутаций ребёнка.
    auto pool = ranked;
    size_t parentIdx;
    for (size_t visited; next.length < EvolutionConfig.populationSize; ++visited)
    {
        enforce(visited < maxParentCycles * pool.length,
            "не удалось заполнить кандидаты физической симуляции: "
            ~ "исчерпаны 3 цикла по родителям");

        const base = pool[parentIdx].chromosome;
        const mate = ranked[tournament(ranked, p.tournamentSize, rnd)].chromosome;
        next ~= tryCreateViableMutant(base.crossover(mate, rnd), p, rnd);
        parentIdx = (parentIdx + 1) % pool.length;
    }
    return next;
}

unittest
{
    import std.random : Random;

    auto rnd = Random(5);
    auto pop = evaluatePopulation(seedPopulation(EvolutionConfig.populationSize));

    // tryCreateViableMutant: из заведомо жизнеспособного родителя живой мутант
    // находится практически всегда, и результат обязан быть годным каркасом.
    foreach (e; pop)
    {
        const auto mutant = tryCreateViableMutant(e.chromosome, EvolutionConfig.init, rnd);
        const auto grown = develop(mutant);
        assert(!grown.isNull && buggyFitness(grown.get) > 0.0f,
            "мутант обязан вырасти в жизнеспособный каркас");
    }

    // Мёртвый родитель не роняет эволюцию: вместо аварии возвращается лучшее
    // из виденного, а отбор такое поколение вычистит по нулевому фитнесу.
    Chromosome dead;
    dead.actProduction = 0.01f;
    dead.inhProduction = 1.0f;
    dead.threshold = 0.02f;
    const auto rescue = tryCreateViableMutant(dead, EvolutionConfig.init, rnd);
    assert(rescue.alleles.length == geneCount, "спасённый ребёнок — тоже хромосома");

    // buildNextGeneration: все слоты заполнены, гены целы.
    auto next = buildNextGeneration(pop, EvolutionConfig.init, rnd);
    assert(next.length == EvolutionConfig.populationSize, "поколение целиком заполнено");
    foreach (chr; next)
        assert(chr.alleles.length == geneCount,
            "кроссовер и мутации сохраняют число генов");

    // Элита: лучший родитель переходит в следующее поколение без изменений.
    Individual[] ranked = pop.dup;
    sort!((a, b) => a.fitness > b.fitness)(ranked);
    assert(next[0].alleles == ranked[0].chromosome.alleles,
        "лучший родитель сохраняется в следующее поколение неизменным");
}