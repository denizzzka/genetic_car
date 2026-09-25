module genetics.selection;

import std.algorithm : max;
import std.random;
import std.stdio : writefln;
import std.parallelism : TaskPool, totalCPUs;

import genetics.sge;
import genetics.buggygrammar;
import genetics.initial_data;
import genetics.fitness;
import genetics.generation : buildNextGeneration;
import physics_world;
import dlib.math.vector;
import frame.frame;

/// Отобранный индивид: геном и его фитнес (0 — невалидный/неразвиваемый).
struct Individual
{
    Genotype genotype;
    float fitness;
}

/// Параметры эволюции — единая точка настройки
struct EvolutionConfig
{
    enum size_t populationSize = 100;
    size_t tournamentSize = 2;
    /// Стартовые темпы мутаций поколения 0; дальше особи самоадаптируются.
    size_t mutateHits = defaultPointHits;
    size_t indelHits = defaultIndelHits;
    float structuralChance = defaultStructuralChance;
    size_t generationsPerPress = 25;

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
Genotype[] seedPopulation(Grammar gr, size_t n,
    const EvolutionConfig params = EvolutionConfig.init)
{
    Genotype[] pop;
    pop.reserve(n);
    auto base = startGenome(gr);
    base.pointHits = params.mutateHits;
    base.indelHits = params.indelHits;
    base.structuralChance = params.structuralChance;
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
                needPhysics ~= new Buggy(placedFrame(may.get.frame));
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
            generation, idx, cast(string) run.why, run.distance, run.wheels, run.beams);
}

/**
 * Прогон `generations` поколений отбора: элитизм + турнирная селекция +
 * кроссовер + мутация. Возвращает оценённую популяцию следующего поколения;
 * исходная не меняется.
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
    auto seed = seedPopulation(gr, EvolutionConfig.populationSize);
    auto pop = evaluatePopulation(gr, seed);
    assert(pop.length == EvolutionConfig.populationSize, "размер популяции сохраняется");
    const f0 = pop[0].fitness;
    assert(f0 > 0.0f, "закодированный багги развивается в валидный каркас");
    foreach (e; pop)
        assert(e.fitness == f0, "поколение 0 — идентичные особи");

    // Элитизм: лучший фитнес не падает при эволюции (элита сохраняется
    // как есть, поэтому верхний фитнес не ниже исходного).
    auto rnd = Random(7);
    auto evolved = evolve(gr, pop, 10, rnd);
    assert(evolved.length == EvolutionConfig.populationSize,
        "поколение строится по enum populationSize");
    assert(bestFitness(evolved) >= f0 - 1e-6f,
        "элитизм гарантирует не хуже исходного лучшего");

    // Родителей больше, чем нужно на поколение, — слоты заполняются
    // гарантированно без провалов мутаций.
    auto many = evaluatePopulation(gr, seedPopulation(gr, EvolutionConfig.populationSize * 2));
    auto rnd3 = Random(7);
    auto evolved3 = evolve(gr, many, 10, rnd3);
    assert(evolved3.length == EvolutionConfig.populationSize,
        "enum populationSize задаёт размер поколения");

    // Тот же посев и та же последовательность — тот же результат.
    auto rnd2 = Random(7);
    auto evolved2 = evolve(gr, evaluatePopulation(gr,
        seedPopulation(gr, EvolutionConfig.populationSize)), 10, rnd2);
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
    auto pop = seedPopulation(gr, EvolutionConfig.populationSize);
    EvolutionConfig cfg;
    cfg.simulateSeconds = 2.0;
    auto e0 = evaluatePopulation(gr, pop, cfg);
    foreach (x; e0)
        assert(isFinite(x.fitness) && x.fitness >= 0.0f, "физика в оценке конечна");
    auto rnd = Random(7);
    auto e1 = evolve(gr, e0, 2, rnd, cfg);
    const float b = bestFitness(e1);
    const float m = meanFitness(e1);
    writeln("PHYS gen2 best=", b, " mean=", m);
    assert(isFinite(b) && isFinite(m) && b >= 0.0f && m >= 0.0f,
        "физический цикл отбора конечен");
}

unittest
{
    // Структурные мутации подключены в цикл отбора: число балок обязано
    // уметь расти за поколения, а не только менять геометрию скелета.
    // Скелет (11 балок) неизменен, поэтому рост меряем относительно него.
    auto gr = buggyGrammar();
    const startAnchors = 2;

    bool grewBeams;
    bool dropAnchors;
    foreach (s; 11 .. 41)
    {
        auto rnd = Random(s);
        auto pop = evaluatePopulation(gr, seedPopulation(gr, 100), EvolutionConfig.init, 0);
        auto evolved = evolve(gr, pop, 16, rnd, EvolutionConfig.init);
        foreach (e; evolved)
            if (auto may = develop(gr, e.genotype))
            {
                if (may.get.frame.beams.length > skeletonBeamCount() + 2)
                    grewBeams = true;
                if (may.get.frame.anchors.length < startAnchors)
                    dropAnchors = true;
            }
    }
    assert(grewBeams, "число балок должно уметь расти через инделы");
    // Жизнеспособные каркасы не теряют колёса (балки и якоря отбор сохраняет);
    // третье колесо требует опоры на внешнюю структуру за зоной кабины —
    // кабину не поощряется трогать колёсами (см. frameCabinContact).
    assert(!dropAnchors, "отбор не должен разоружать багги до одного колеса");
}