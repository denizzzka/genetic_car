module genetics.selection;

import std.algorithm : sort, min, max;
import std.random;

import genetics.sge;
import genetics.buggygrammar;
import genetics.encoder;
import genetics.fitness;

/// Отобранный индивид: геном и его фитнес (0 — невалидный/неразвиваемый).
struct Individual
{
    Genotype genotype;
    float fitness;
}

/// Параметры отбора.
///
/// Значения подобраны так, чтобы витрина из топ-5 оставалась разнообразной:
/// слабый турнир (2) и мизерная элита (1) не дают популяции схлопнуться в
/// клоны одной особи, а мутация покрупнее (3 кодона) продолжает разведку.
struct SelectionParams
{
    size_t populationSize = 20;
    size_t tournamentSize = 2;
    size_t eliteCount = 1;
    size_t mutateHits = 3;
}

/// Поколение 0: идентичные копии закодированного дефолтного багги.
///
/// Все особи одного генома — первая галерея стоит одинаковой шеренгой,
/// а мутация раздробит её уже на первом поколении. Так различие между
/// «до» и «после» отбора видно сразу.
Genotype[] seedPopulation(Grammar gr, size_t n)
{
    Genotype[] pop;
    pop.reserve(n);
    const base = startGenome(gr);
    foreach (_; 0 .. n)
        pop ~= base.dup;
    return pop;
}

/// Оценка популяции: develop + buggyFitness. Неразвиваемый геном — 0.
Individual[] evaluatePopulation(const Grammar gr, Genotype[] pop)
{
    Individual[] res;
    res.reserve(pop.length);
    foreach (g; pop)
    {
        float fit = 0.0f;
        auto may = develop(gr, g);
        if (!may.isNull)
            fit = buggyFitness(may.get);
        res ~= Individual(g, fit);
    }
    return res;
}

/**
 * Прогон `generations` поколений отбора: элитизм + турнирная селекция +
 * кроссовер + мутация. Возвращает оценённую популяцию следующего
 * поколения; исходная не меняется.
 */
Individual[] evolve(const Grammar gr, Individual[] pop,
    size_t generations, ref Random rnd, SelectionParams params = SelectionParams.init)
{
    auto cur = pop;
    foreach (_; 0 .. generations)
    {
        auto children = buildNextGeneration(cur, params, rnd);
        cur = evaluatePopulation(gr, children);
    }
    return cur;
}

private Genotype[] buildNextGeneration(Individual[] pop,
    const SelectionParams p, ref Random rnd)
{
    Genotype[] next;
    next.reserve(p.populationSize);

    // Копия с сортировкой по убыванию — элита и родители из неё.
    // (явное копирование из-за поля-ссылки: array.dup не применим)
    Individual[] ranked = new Individual[pop.length];
    foreach (i, e; pop)
        ranked[i] = e;
    sort!((a, b) => a.fitness > b.fitness)(ranked);

    const elites = min(p.eliteCount, ranked.length);
    foreach (i; 0 .. elites)
        next ~= ranked[i].genotype.dup;

    while (next.length < p.populationSize)
    {
        const pa = ranked[tournament(ranked, p.tournamentSize, rnd)].genotype;
        const pb = ranked[tournament(ranked, p.tournamentSize, rnd)].genotype;
        auto child = crossover(pa, pb, rnd);
        mutate(child, p.mutateHits, rnd);
        next ~= child;
    }
    return next;
}

/// Турнирная селекция: индекс особи с максимальным фитнесом среди `k`.
size_t tournament(const Individual[] pop, size_t k, ref Random rnd)
{
    auto best = uniform(0, pop.length, rnd);
    foreach (_; 1 .. k)
    {
        const idx = uniform(0, pop.length, rnd);
        if (pop[idx].fitness > pop[best].fitness)
            best = idx;
    }
    return best;
}

float bestFitness(const Individual[] pop)
{
    float best = 0.0f;
    foreach (e; pop)
        best = max(best, e.fitness);
    return best;
}

float meanFitness(const Individual[] pop)
{
    if (pop.length == 0)
        return 0.0f;
    float sum = 0.0f;
    foreach (e; pop)
        sum += e.fitness;
    return sum / pop.length;
}

unittest
{
    // Поколение 0: все геномы идентичны, фитнес одинаковый и положительный.
    auto gr = buggyGrammar();
    auto seed = seedPopulation(gr, 20);
    auto pop = evaluatePopulation(gr, seed);
    assert(pop.length == 20, "размер популяции сохраняется");
    const f0 = pop[0].fitness;
    assert(f0 > 0.0f, "закодированный багги развивается в валидный каркас");
    foreach (e; pop)
        assert(e.fitness == f0, "поколение 0 — идентичные особи");

    // Элитизм: лучший фитнес не падает при эволюции.
    auto rnd = Random(7);
    auto evolved = evolve(gr, pop, 10, rnd);
    assert(evolved.length == 20, "популяция не меняет размер");
    assert(bestFitness(evolved) >= f0 - 1e-6f,
        "элитизм гарантирует не хуже исходного лучшего");

    // Тот же посев и та же последовательность — тот же результат.
    auto rnd2 = Random(7);
    auto evolved2 = evolve(gr, evaluatePopulation(gr, seedPopulation(gr, 20)),
        10, rnd2);
    assert(bestFitness(evolved2) == bestFitness(evolved),
        "отбор детерминирован при фиксированном зерне");
}