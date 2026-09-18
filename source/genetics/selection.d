module genetics.selection;

import std.algorithm : sort, min, max;
import std.random;

import genetics.sge;
import genetics.buggygrammar;
import genetics.buggyast;
import genetics.initial_data;
import genetics.fitness;

/// Отобранный индивид: геном и его фитнес (0 — невалидный/неразвиваемый).
struct Individual
{
    Genotype genotype;
    float fitness;
}

/// Параметры эволюции — единая точка настройки
struct EvolutionConfig
{
    size_t populationSize = 100;
    size_t tournamentSize = 2;
    size_t eliteCount = 5;
    size_t mutateHits = 3;
    size_t generationsPerPress = 100;
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
        Ast ast;
        auto may = develop(gr, g, ast);
        if (!may.isNull)
            fit = buggyFitness(may.get, ast);
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
    size_t generations, ref Random rnd, EvolutionConfig params = EvolutionConfig.init)
{
    auto cur = pop;
    foreach (_; 0 .. generations)
    {
        auto children = buildNextGeneration(gr, cur, params, rnd);
        cur = evaluatePopulation(gr, children);
    }
    return cur;
}

private Genotype[] buildNextGeneration(const Grammar gr, Individual[] pop,
    const EvolutionConfig p, ref Random rnd)
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

        // Шаг мутации — mutateStep: с вероятностью 25% это индел — единственный
        // оператор, меняющий число балок и колёс. Раньше звался только mutate,
        // геномы не меняли длину, и эволюция лишь сдвигала геометрию фиксированного
        // скелета. mutateStep принимает кандидата только если индел реально изменил
        // структуру; если подходящий шаг не нашёлся — запасной точечный mutate.
        auto may = mutateStep(gr, child, rnd);
        if (may.isNull)
            mutate(child, p.mutateHits, rnd);
        else
            child = may.get;
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
    assert(evolved.length == EvolutionConfig.init.populationSize,
        "поколение строится по EvolutionConfig.populationSize");
    assert(bestFitness(evolved) >= f0 - 1e-6f,
        "элитизм гарантирует не хуже исходного лучшего");

    // Иная популяция по явному конфигу.
    EvolutionConfig cfg;
    cfg.populationSize = 20;
    auto rnd3 = Random(7);
    auto evolved3 = evolve(gr, pop, 10, rnd3, cfg);
    assert(evolved3.length == 20, "явный конфиг задаёт размер поколения");

    // Тот же посев и та же последовательность — тот же результат.
    auto rnd2 = Random(7);
    auto evolved2 = evolve(gr, evaluatePopulation(gr, seedPopulation(gr, 20)),
        10, rnd2);
    assert(bestFitness(evolved2) == bestFitness(evolved),
        "отбор детерминирован при фиксированном зерне");
}

unittest
{
    // Структурные мутации подключены в цикл отбора (mutateStep): число
    // балок и колёс обязано уметь расти за поколения, а не только менять
    // геометрию фиксированного скелета.
    auto gr = buggyGrammar();
    const startBeams = 2;
    const startAnchors = 2;
    auto rnd = Random(11);
    auto pop = evaluatePopulation(gr, seedPopulation(gr, 20));
    auto evolved = evolve(gr, pop, 8, rnd);

    bool grewBeams, grewAnchors;
    foreach (e; evolved)
    {
        auto may = develop(gr, e.genotype);
        if (may.isNull)
            continue;
        if (may.get.beams.length > startBeams)
            grewBeams = true;
        if (may.get.anchors.length > startAnchors)
            grewAnchors = true;
    }
    assert(grewBeams, "число балок должно уметь расти через инделы");
    assert(grewAnchors, "число колёс должно уметь расти через инделы");
}