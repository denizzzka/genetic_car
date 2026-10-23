module genetics.generation;

import std.algorithm : sort, map, min, max;
import std.format : format;
import std.math : sqrt;
import std.array : array;
import std.exception : enforce;
import std.range : iota;
import std.random;
import std.typecons : Nullable;

import frame.frame : Frame;
import genetics.chromosome : Chromosome, alleleCount, alleles, crossover, mutated;
import genetics.fitness : buggyFitness;
import genetics.growth : Organism, develop;
import genetics.selection;

private enum float elitePercent = 5.0f;

/// Попыток мутации одного ребёнка, прежде чем он считается провальным.
private enum size_t maxMutationAttempts = 30;

/// Полных проходов по массиву родителей, после которых
/// незаполненные слоты поколения — повод для `enforce`.
private enum size_t maxParentCycles = 3;

/// Что дала мутация одного ребёнка. Считается только при включённой
/// статистике: иначе поля остаются нулями и ничего не стоит.
struct MutationStats
{
    size_t attempts;   // проб развития
    size_t clones;     // вернулся родитель без изменений
    size_t viable;     // первый же годный мутант
    size_t fallback;   // годного нет, взят лучший негодный
    size_t genes;      // сколько аллелей отличается от родителя

    void add(const MutationStats other)
    {
        attempts += other.attempts;
        clones += other.clones;
        viable += other.viable;
        fallback += other.fallback;
        genes += other.genes;
    }
}

/// Сводка поколения: мутации при сборке и фитнес после заездов.
struct GenerationStats
{
    MutationStats mutation;
    size_t individuals;
    size_t unique;
    float best;
    float mean;
    float variance;

    /// Наследуемая изменчивость популяции: среднее, разброс и пределы
    /// гена mutationRate.
    float rateMean;
    float rateSd;
    float rateMin;
    float rateMax;

    void addPopulation(const Individual[] pop)
    {
        individuals = pop.length;
        if (individuals == 0)
            return;

        best = bestFitness(pop);
        mean = meanFitness(pop);
        float sum = 0.0f;
        foreach (e; pop)
        {
            const float d = e.fitness - mean;
            sum += d * d;
        }
        variance = sum / individuals;

        float rateSum = 0.0f;
        rateMin = float.max;
        rateMax = -float.max;
        foreach (e; pop)
        {
            const float r = e.chromosome.mutationRate;
            rateSum += r;
            rateMin = min(rateMin, r);
            rateMax = max(rateMax, r);
        }
        rateMean = rateSum / individuals;
        float rateVar = 0.0f;
        foreach (e; pop)
        {
            const float d = e.chromosome.mutationRate - rateMean;
            rateVar += d * d;
        }
        rateSd = sqrt(rateVar / individuals);

        unique = 0;
        foreach (i, e; pop)
        {
            bool seen;
            foreach (j; 0 .. i)
                if (alleles(pop[j].chromosome) == alleles(e.chromosome))
                {
                    seen = true;
                    break;
                }
            if (!seen)
                ++unique;
        }
    }

    string line() const
    {
        const float n = individuals > 0 ? cast(float) individuals : 1.0f;
        return format("clones=%d viable=%d fallback=%d attempts=%.1f genes=%.1f unique=%d/%d best=%.4f mean=%.4f var=%.5f rate=%.5f±%.5f [%.5f..%.5f]",
            mutation.clones, mutation.viable, mutation.fallback,
            mutation.attempts / n, mutation.genes / n,
            unique, individuals, best, mean, variance,
            rateMean, rateSd, rateMin, rateMax);
    }
}

/// Сколько аллелей различаются, включая разницу длин списков фаз.
private size_t alleleDistance(const Chromosome a, const Chromosome b)
{
    const aa = alleles(a);
    const bb = alleles(b);
    const size_t n = min(aa.length, bb.length);
    size_t d = max(aa.length, bb.length) - n;
    foreach (i; 0 .. n)
        if (aa[i] != bb[i])
            ++d;
    return d;
}

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
    ref Random rnd, Nullable!Frame baseFrame = Nullable!Frame.init,
    MutationStats* stats = null)
{
    Chromosome best = base.clone;
    Nullable!Frame bestFrame = baseFrame;
    size_t bestAnchors = baseFrame.isNull ? 0 : baseFrame.get.anchors.length;
    size_t attempts;

    foreach (_; 0 .. maxMutationAttempts)
    {
        ++attempts;
        auto trial = mutated(base, base.mutationRate, rnd);
        // Одна проба — один вырост: и годность, и число якорей берём из него же.
        auto grownFrame = develop(trial);
        if (grownFrame.isNull)
            continue;
        auto frame = grownFrame.get;
        if (buggyFitness(frame) > 0.0f)
        {
            if (stats !is null)
            {
                stats.attempts = attempts;
                stats.viable = 1;
                stats.genes = alleleDistance(trial, base);
            }
            return Organism(trial, Nullable!Frame(frame));
        }
        if (frame.anchors.length > bestAnchors)
        {
            best = trial;
            bestFrame = Nullable!Frame(frame);
            bestAnchors = frame.anchors.length;
        }
    }
    if (stats !is null)
    {
        stats.attempts = attempts;
        stats.genes = alleleDistance(best, base);
        if (stats.genes == 0)
            stats.clones = 1;
        else
            stats.fallback = 1;
    }
    return Organism(best, bestFrame);
}

/// Построение следующего поколения — заполнение слотов физической симуляции.
///
/// Слоты заполняются, пока хватает бюджета `maxParentCycles` циклов по
/// родителям; когда бюджет исчерпан, а слоты не заполнены — `enforce`.
/// Каждый слот отдаёт `Organism` — каркас выращен попутно с проверкой годности.
Organism[] buildNextGeneration(Individual[] pop, const EvolutionConfig p,
    ref Random rnd, GenerationStats* stats = null)
{
    if (stats !is null)
        *stats = GenerationStats.init;

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
            kids[k] = crossover(base.chromosome, mate, rnd);
            baseFrames[k] = base.frame;
            kidsRnd[k] = Random(uniform!uint(rnd));
            parentIdx = (parentIdx + 1) % pool.length;
        }

        Organism[] grown = new Organism[batch];
        MutationStats[] mutStats;
        if (stats !is null)
            mutStats = new MutationStats[batch];
        foreach (k; workPool().parallel(iota(0, batch), 1))
            grown[k] = tryCreateViableMutant(kids[k], p, kidsRnd[k], baseFrames[k],
                mutStats.length > 0 ? &mutStats[k] : null);
        if (stats !is null)
            foreach (s; mutStats)
                stats.mutation.add(s);
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
    Chromosome dead = Chromosome.init.clone;
    dead.segments[0].actProduction = 0.01f;
    dead.segments[0].inhProduction = 1.0f;
    dead.segments[0].threshold = 0.02f;
    const auto rescue = tryCreateViableMutant(dead, EvolutionConfig.init, rnd);
    assert(rescue.chromosome.alleles.length == rescue.chromosome.alleleCount,
        "спасённый ребёнок — тоже хромосома");

    // buildNextGeneration: все слоты заполнены, гены целы.
    auto next = buildNextGeneration(pop, EvolutionConfig.init, rnd);
    assert(next.length == EvolutionConfig.populationSize, "поколение целиком заполнено");
    foreach (org; next)
        assert(org.chromosome.alleles.length == org.chromosome.alleleCount,
            "кроссовер и мутации сохраняют число генов");

    // Элита: лучший родитель переходит в следующее поколение без изменений.
    Individual[] ranked = pop.dup;
    sort!((a, b) => a.fitness > b.fitness)(ranked);
    assert(next[0].chromosome.alleles == ranked[0].chromosome.alleles,
        "лучший родитель сохраняется в следующее поколение неизменным");
}