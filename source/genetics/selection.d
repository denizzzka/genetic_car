module genetics.selection;

import std.algorithm : sort, min, max;
import std.random;
import std.stdio : writefln;
import std.parallelism : TaskPool, totalCPUs;

import frame.frame;
import genetics.sge;
import genetics.buggygrammar;
import genetics.initial_data;
import genetics.fitness;
import physics_world;
import dlib.math.vector;

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
    size_t generationsPerPress = 50;

    double simulateSeconds = 0.0;

    /// Печатать в stdout итоги физического заезда по каждой особи
    /// (`physicsFitness`) и сводку best/mean по каждому поколению `evolve`.
    /// По умолчанию тихо — включается во вьюере для наблюдения за эволюцией.
    bool logPhysics = false;
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

/// Пул Phobos для физических заездов поколения, не более 75% ядер.
/// Демон-воркеры: на выходе их убирает статический деструктор Phobos.
private __gshared TaskPool physicsPool_;

private TaskPool physicsPool()
{
    if (physicsPool_ is null)
    {
        const n = max(1, (cast(size_t) totalCPUs * 3) / 4);
        physicsPool_ = new TaskPool(n);
        physicsPool_.isDaemon = true;
    }
    return physicsPool_;
}

struct PhysicsBatch
{
    Individual[] res;
    Buggy[] needPhysics;
    size_t[] physIdx;
}

PhysicsBatch evaluateStatic(const Grammar gr, Genotype[] pop,
    const EvolutionConfig params = EvolutionConfig.init)
{
    auto res = new Individual[pop.length];

    Buggy[] needPhysics;
    needPhysics.reserve(pop.length);
    size_t[] physIdx;
    physIdx.reserve(pop.length);

    foreach (i, g; pop)
    {
        float fit = 0.0f;
        if (auto may = develop(gr, g))
        {
            fit = buggyFitness(may.get.frame, may.get.ast);
            if (fit > 0.0f && params.simulateSeconds > 0.0)
            {
                needPhysics ~= new Buggy(may.get.frame, vec3(0.0f));
                physIdx ~= i;
            }
        }
        res[i] = Individual(g, fit);
    }

    return PhysicsBatch(res, needPhysics, physIdx);
}

void runPhysics(ref PhysicsBatch batch, const EvolutionConfig params,
    size_t generation = 0)
{
    const runs = physicsPool().amap!runBuggy(batch.needPhysics);
    foreach (k, run; runs)
    {
        batch.res[batch.physIdx[k]].fitness *= run.score;
        if (params.logPhysics)
            logPhysicsIndividual(batch.physIdx[k], generation,
                batch.res[batch.physIdx[k]].fitness, run);
    }
}

/// Оценка популяции
Individual[] evaluatePopulation(const Grammar gr, Genotype[] pop,
    const EvolutionConfig params = EvolutionConfig.init, size_t generation = 0)
{
    auto batch = evaluateStatic(gr, pop, params);
    runPhysics(batch, params, generation);
    return batch.res;
}

private PhysicsResult runBuggy(Buggy buggy)
{
    return physicsFitness(buggy, physicsSimSeconds);
}

private void logPhysicsIndividual(size_t idx, size_t generation, float finalFit, const PhysicsResult run)
{
    if (run.survived)
        writefln("gen %d: #%d fit=%.4f score=%.3f roll=%.1fm wheels=%d beams=%d",
            generation, idx, finalFit, run.score, run.distance, run.wheels, run.beams);
    else
        writefln("gen %d: #%d FAILED (%s) roll=%.1fm wheels=%d beams=%d",
            generation, idx, run.why, run.distance, run.wheels, run.beams);
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
    foreach (gen; 0 .. generations)
    {
        auto children = buildNextGeneration(gr, cur, params, rnd);
        auto batch = evaluateStatic(gr, children, params);
        runPhysics(batch, params, gen + 1);
        cur = batch.res;
        if (params.logPhysics)
            writefln("gen %2d: best=%.4f mean=%.4f",
                gen + 1, bestFitness(cur), meanFitness(cur));
    }
    return cur;
}

/// Правок пробуем, прежде чем откатиться к исходному геному.
private enum size_t maxMutationAttempts = 24;

/**
 * Мутация с подбором: кандидат каждый раз заново мутируется из исходного
 * генома и принимается, только если развивается в валидный каркас — `develop`
 * даёт ненулевой результат (в конце он проходит `isValidFrame`).
 *
 * С вероятностью 25% вызов целиком — подбор индела (единственного оператора,
 * меняющего число балок и колёс); такой кандидат принимается, только если
 * реально изменил структуру — иначе редкая структурная правка тонет среди
 * геометрических, и каркас не растёт. Иначе весь вызов — точечные мутации.
 * После `maxMutationAttempts` неудачных попыток — откат к исходному
 * (немутированному) геному.
 */
private Genotype mutateChecked(const Grammar gr, const Genotype base,
    size_t mutateHits, ref Random rnd)
{
    auto baseDev = develop(gr, base);
    const baseBeams = baseDev.isNull ? size_t.max : baseDev.get.frame.beams.length;
    const baseAnchors = baseDev.isNull ? size_t.max : baseDev.get.frame.anchors.length;

    const structural = uniform(0.0f, 1.0f, rnd) < 0.25f;
    foreach (_; 0 .. maxMutationAttempts)
    {
        auto candidate = base.dup;
        if (structural)
            mutateIndel(candidate, 1, rnd);
        else
            mutate(candidate, mutateHits, rnd);

        auto may = develop(gr, candidate);
        if (may.isNull)
            continue;
        if (structural
            && may.get.frame.beams.length == baseBeams
            && may.get.frame.anchors.length == baseAnchors)
            continue;
        return candidate;
    }
    return base.dup;
}

Genotype[] buildNextGeneration(const Grammar gr, Individual[] pop,
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

        // Мутация с проверкой валидности каркаса: кандидат заново мутируется
        // из исходного ребёнка, пока развивка не даёт валидный каркас;
        // после maxMutationAttempts — откат к исходному ребёнку.
        next ~= mutateChecked(gr, child, p.mutateHits, rnd);
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
    // Физический слой в цикле отбора: оценка и эволюция с simulateSeconds
    // должны быть конечными и не разваливаться. Величина счёта зависит от
    // каркаса — здесь важно отсутствие NaN/разлёта по поколениям.
    import std.stdio : writeln;
    import std.math : isFinite;
    auto gr = buggyGrammar();
    auto pop = seedPopulation(gr, 8);
    EvolutionConfig cfg;
    cfg.populationSize = 8;
    cfg.simulateSeconds = 2.0;
    auto e0 = evaluatePopulation(gr, pop, cfg);
    foreach (x; e0)
        assert(isFinite(x.fitness) && x.fitness >= 0.0f, "физика в оценке конечна");
    auto rnd = Random(7);
    auto e1 = evolve(gr, e0, 5, rnd, cfg);
    const float b = bestFitness(e1);
    const float m = meanFitness(e1);
    writeln("PHYS gen5 best=", b, " mean=", m);
    assert(isFinite(b) && isFinite(m) && b >= 0.0f && m >= 0.0f,
        "физический цикл отбора конечен");
}

unittest
{
    // Структурные мутации подключены в цикл отбора (mutateChecked): число
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
        if (may.get.frame.beams.length > startBeams)
            grewBeams = true;
        if (may.get.frame.anchors.length > startAnchors)
            grewAnchors = true;
    }
    assert(grewBeams, "число балок должно уметь расти через инделы");
    assert(grewAnchors, "число колёс должно уметь расти через инделы");
}

unittest
{
    // mutateChecked: результат либо развивается в валидный каркас, либо —
    // откат к исходному (валидному) геному после исчерпания попыток.
    auto gr = buggyGrammar();
    auto rnd = Random(5);
    auto pop = evaluatePopulation(gr, seedPopulation(gr, 20));
    foreach (e; pop)
    {
        auto child = mutateChecked(gr, e.genotype, 3, rnd);
        assert(!develop(gr, child).isNull || child.genes == e.genotype.genes,
            "мутация с подбором либо валидна, либо откатывается к исходнику");
    }
}