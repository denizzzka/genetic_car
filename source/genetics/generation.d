module genetics.generation;

import std.algorithm : sort, map, min, max;
import std.array : array;
import std.exception : enforce;
import std.range : iota;
import std.random;
import std.typecons : Nullable;

import frame.frame : Frame;
import genetics.chromosome : Chromosome, geneCount;
import genetics.fitness : buggyFitness;
import genetics.growth : Organism, develop;
import genetics.selection;

private enum float elitePercent = 5.0f;

/// Попыток мутации одного ребёнка, прежде чем он считается провальным.
private enum size_t maxMutationAttempts = 30;

/// Полных проходов по массиву родителей, после которых
/// незаполненные слоты поколения — повод для `enforce`.
private enum size_t maxParentCycles = 3;

/**
 * Годный ребёнок `base` для слотов будущей физической симуляции — вместе с
 * уже выращенным каркасом, который отбору незачем растить заново.
 *
 * Мутация всегда берётся от `base`, а не от предыдущей пробы: накапливающийся
 * шаг мутаций иначе быстро доводит геном до края диапазона, где машина уже
 * не вырастает. Проб до `maxMutationAttempts` достаточно: при
 * заметной доле выживающих родитель всегда даёт годного ребёнка.
 *
 * Если годного ребёнка нет всё же нет, возвращается лучшее из виденного —
 * родитель или самая «колёсистая» проба. Отбор сам отбракует такой каркас по
 * нулевому фитнесу, но эволюция не встанет колом.
 */
Organism tryCreateViableMutant(const Chromosome base, const EvolutionConfig p,
    ref Random rnd, Nullable!Frame baseFrame = Nullable!Frame.init)
{
    Chromosome best = base;
    Nullable!Frame bestFrame = baseFrame;
    size_t bestAnchors = baseFrame.isNull ? 0 : baseFrame.get.anchors.length;

    foreach (_; 0 .. maxMutationAttempts)
    {
        auto trial = base.mutated(p.mutationRate, rnd);
        // Одна проба — один вырост: и годность, и число якорей берём из него же.
        auto grownFrame = develop(trial);
        if (grownFrame.isNull)
            continue;
        auto frame = grownFrame.get;
        if (buggyFitness(frame) > 0.0f)
            return Organism(trial, Nullable!Frame(frame));
        if (frame.anchors.length > bestAnchors)
        {
            best = trial;
            bestFrame = Nullable!Frame(frame);
            bestAnchors = frame.anchors.length;
        }
    }
    return Organism(best, bestFrame);
}

/// Построение следующего поколения — заполнение слотов физической симуляции.
///
/// Слоты заполняются, пока хватает бюджета `maxParentCycles` циклов по
/// родителям; когда бюджет исчерпан, а слоты не заполнены — `enforce`.
/// Каждый слот отдаёт `Organism` — каркас выращен попутно с проверкой годности.
Organism[] buildNextGeneration(Individual[] pop, const EvolutionConfig p,
    ref Random rnd)
{
    assert(pop.length > 0,
        "не из чего строить следующее поколение: родителей нет");

    auto ranked = pop.dup.sort!((a, b) => a.fitness > b.fitness).release;

    Organism[] next;
    next.reserve(EvolutionConfig.populationSize);

    // Элита: лучшие elitePercent% родителей переходят без изменений.
    const elites = min(EvolutionConfig.populationSize,
        max(cast(size_t) 1,
            cast(size_t) (ranked.length * elitePercent / 100.0f)));
    next ~= ranked[0 .. elites].map!(e => Organism(e.chromosome, e.frame)).array;

    // Заполнение остальных слотов: родители — все, лучшие первыми, по ним
    // счётчик идёт по кругу; на каждого — кроссовер со случайным партнёром
    // (турнир по всей популяции) и до maxMutationAttempts мутаций ребёнка.
    //
    // Рост детей одного круга идёт на пуле: родители в круге независимы, а
    // выращивание каркаса — самая тяжёлая часть поколения. Выбор родителя,
    // партнёра и кроссовер остаются на главном потоке: это дешёво, зато RNG
    // не надо делить между воркерами — каждому ребёнку достаётся своя соль.
    auto pool = ranked;
    size_t parentIdx;
    for (size_t visited; next.length < EvolutionConfig.populationSize;)
    {
        const size_t budget = maxParentCycles * pool.length;
        enforce(visited < budget,
            "не удалось заполнить кандидаты физической симуляции: "
            ~ "исчерпаны 3 цикла по родителям");
        const size_t batch = min(pool.length,
            min(EvolutionConfig.populationSize - next.length, budget - visited));

        Chromosome[] kids = new Chromosome[batch];
        Nullable!Frame[] baseFrames = new Nullable!Frame[batch];
        Random[] kidsRnd = new Random[batch];
        foreach (k; 0 .. batch)
        {
            auto base = pool[parentIdx];
            const mate = ranked[tournament(ranked, p.tournamentSize, rnd)].chromosome;
            kids[k] = base.chromosome.crossover(mate, rnd);
            baseFrames[k] = base.frame;
            kidsRnd[k] = Random(uniform!uint(rnd));
            parentIdx = (parentIdx + 1) % pool.length;
        }

        Organism[] grown = new Organism[batch];
        foreach (k; workPool().parallel(iota(0, batch), 1))
            grown[k] = tryCreateViableMutant(kids[k], p, kidsRnd[k], baseFrames[k]);
        next ~= grown;
        visited += batch;
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
        const auto grown = develop(mutant.chromosome);
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
    assert(rescue.chromosome.alleles.length == geneCount,
        "спасённый ребёнок — тоже хромосома");

    // buildNextGeneration: все слоты заполнены, гены целы.
    auto next = buildNextGeneration(pop, EvolutionConfig.init, rnd);
    assert(next.length == EvolutionConfig.populationSize, "поколение целиком заполнено");
    foreach (org; next)
        assert(org.chromosome.alleles.length == geneCount,
            "кроссовер и мутации сохраняют число генов");

    // Элита: лучший родитель переходит в следующее поколение без изменений.
    Individual[] ranked = pop.dup;
    sort!((a, b) => a.fitness > b.fitness)(ranked);
    assert(next[0].chromosome.alleles == ranked[0].chromosome.alleles,
        "лучший родитель сохраняется в следующее поколение неизменным");
}