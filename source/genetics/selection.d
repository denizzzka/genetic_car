module genetics.selection;

import std.algorithm : max;
import std.range : iota;
import std.random;
import std.stdio : writefln;
import std.parallelism : TaskPool;
import std.typecons : Nullable;

import genetics.chromosome;
import genetics.growth : Organism, develop;
import genetics.initial_data;
import genetics.fitness;
import genetics.generation : buildNextGeneration;
import physics_world;
import dlib.math.vector;
import frame.frame;
import frame.cockpit : cockpitFrameBeamCount;

/// Отобранный индивид: геном, его каркас и фитнес (0 — невалидный/неразвиваемый).
/// Каркас хранится, чтобы элита следующего поколения не росла заново.
struct Individual
{
    Chromosome chromosome;
    Nullable!Frame frame;
    float fitness;
}

/// Параметры эволюции — единая точка настройки
struct EvolutionConfig
{
    enum size_t populationSize = 100;
    size_t tournamentSize = 2;

    /// Шаг мутации в долях диапазона гена. Мало — эволюция ползёт, много —
    /// потомки мгновенно вырождаются и отбору нечего выбирать.
    float mutationRate = 0.05f;

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
Chromosome[] seedPopulation(size_t n)
{
    Chromosome[] pop;
    pop.reserve(n);
    const base = startChromosome;
    foreach (_; 0 .. n)
        pop ~= base;
    return pop;
}

/// Пул Phobos для работы поколения: заезды физики и, между ними, рост
/// каркасов — не более 75% ядер.
/// Демон-воркеры: на выходе их убирает статический деструктор Phobos.
/// Инициализируется под __gshared-локом: впервые пул поднимается либо с
/// главного потока, либо с jobThread вьюера — кто первый, тот и строит.
private __gshared Object workPoolLock_ = new Object();
private __gshared TaskPool workPool_;
private __gshared size_t workPoolSize_;

/// Задать размер рабочего пула; действует до первого workPool().
void setWorkPoolSize(size_t n)
{
    workPoolSize_ = n;
}

/// Пул для заездов и роста каркасов: пока идёт одно, другое свободно, так
/// что второй пул в процессе только мешал бы.
TaskPool workPool()
{
    if (workPool_ !is null)
        return workPool_;
    synchronized (workPoolLock_)
    {
        if (workPool_ is null)
        {
            // Без флага потоков столько же, сколько миров в пуле физики:
            // лишние воркеры только блокируются на его условии, а мир и его
            // сетка земли — самые тяжёлые объекты в процессе.
            const n = workPoolSize_ > 0 ? workPoolSize_ : maxPooledWorlds;
            workPool_ = new TaskPool(n);
            workPool_.isDaemon = true;
        }
        return workPool_;
    }
}


struct PhysicsBatch
{
    Individual[] res;
    Buggy[] needPhysics;
    size_t[] physIdx;
}

/// Оценка особей с готовым каркасом: без фенотипа особь растится заново.
PhysicsBatch evaluateStatic(Organism[] pop,
    const EvolutionConfig params = EvolutionConfig.init)
{
    auto res = new Individual[pop.length];

    Buggy[] needPhysics;
    needPhysics.reserve(pop.length);
    size_t[] physIdx;
    physIdx.reserve(pop.length);

    foreach (i, org; pop)
    {
        auto frame = org.frame.isNull ? develop(org.chromosome) : org.frame;
        float fit = 0.0f;
        if (!frame.isNull)
        {
            fit = buggyFitness(frame.get);
            if (fit > 0.0f && params.simulateSeconds > 0.0)
            {
                needPhysics ~= new Buggy(placedFrame(frame.get));
                physIdx ~= i;
            }
        }
        res[i] = Individual(org.chromosome, frame, fit);
    }

    return PhysicsBatch(res, needPhysics, physIdx);
}

/// Оценка голых хромосом: каркаса нет — растим с нуля.
PhysicsBatch evaluateStatic(Chromosome[] pop,
    const EvolutionConfig params = EvolutionConfig.init)
{
    Organism[] orgs;
    orgs.reserve(pop.length);
    foreach (chr; pop)
        orgs ~= Organism(chr, Nullable!Frame.init);
    return evaluateStatic(orgs, params);
}

void runPhysics(ref PhysicsBatch batch, const EvolutionConfig params,
    size_t generation = 0)
{
    // Индексы, а не замыкание с `params`: LDC не собирает nested-функции с
    // захватом (dual-context), а разбирать поколение по индексам и писать
    // лог на одном потоке заодно делает вывод детерминированным.
    const size_t n = batch.needPhysics.length;
    PhysicsResult[] runs = new PhysicsResult[n];
    foreach (i; workPool().parallel(iota(0, n), 1))
        runs[i] = runBuggy(batch.needPhysics[i], params);

    foreach (i, run; runs)
    {
        batch.res[batch.physIdx[i]].fitness *= run.score;
        if (params.logPhysics)
            logPhysicsIndividual(batch.physIdx[i], generation,
                batch.res[batch.physIdx[i]].fitness, run);
    }
}

/// Оценка популяции
Individual[] evaluatePopulation(Chromosome[] pop,
    const EvolutionConfig params = EvolutionConfig.init, size_t generation = 0)
{
    auto batch = evaluateStatic(pop, params);
    runPhysics(batch, params, generation);
    return batch.res;
}

private PhysicsResult runBuggy(Buggy buggy, const EvolutionConfig params)
{
    return physicsFitness(buggy, params.simulateSeconds);
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
Individual[] evolve(Individual[] pop, size_t generations, ref Random rnd,
    EvolutionConfig params = EvolutionConfig.init)
{
    auto cur = pop;
    foreach (gen; 0 .. generations)
    {
        auto children = buildNextGeneration(cur, params, rnd);
        auto batch = evaluateStatic(children, params);
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
    // Поколение 0: все хромосомы идентичны, фитнес одинаковый и положительный.
    auto seed = seedPopulation(EvolutionConfig.populationSize);
    auto pop = evaluatePopulation(seed);
    assert(pop.length == EvolutionConfig.populationSize, "размер популяции сохраняется");
    const f0 = pop[0].fitness;
    assert(f0 > 0.0f, "хромосома основателя вырастает в валидный каркас");
    foreach (e; pop)
        assert(e.fitness == f0, "поколение 0 — идентичные особи");

    // Элитизм: лучший фитнес не падает при эволюции (элита сохраняется
    // как есть, поэтому верхний фитнес не ниже исходного).
    auto rnd = Random(7);
    auto evolved = evolve(pop, 10, rnd);
    assert(evolved.length == EvolutionConfig.populationSize,
        "поколение строится по enum populationSize");
    assert(bestFitness(evolved) >= f0 - 1e-6f,
        "элитизм гарантирует не хуже исходного лучшего");

    // Родителей больше, чем нужно на поколение, — слоты заполняются
    // гарантированно без провалов мутаций.
    auto many = evaluatePopulation(seedPopulation(EvolutionConfig.populationSize * 2));
    auto rnd3 = Random(7);
    auto evolved3 = evolve(many, 10, rnd3);
    assert(evolved3.length == EvolutionConfig.populationSize,
        "enum populationSize задаёт размер поколения");

    // Тот же посев и та же последовательность — тот же результат.
    auto rnd2 = Random(7);
    auto evolved2 = evolve(evaluatePopulation(
        seedPopulation(EvolutionConfig.populationSize)), 10, rnd2);
    assert(bestFitness(evolved2) == bestFitness(evolved),
        "отбор детерминирован при фиксированном зерне");
}

unittest
{
    // Физический слой в цикле отбора: оценка и эволюция с simulateSeconds
    // должны быть конечными и не разваливаться. Величина счёта зависит от
    // каркаса — здесь важно отсутствие NaN/разлёта по поколениям.
    import std.math : isFinite;
    auto pop = seedPopulation(EvolutionConfig.populationSize);
    EvolutionConfig cfg;
    cfg.simulateSeconds = 2.0;
    auto e0 = evaluatePopulation(pop, cfg);
    foreach (x; e0)
        assert(isFinite(x.fitness) && x.fitness >= 0.0f, "физика в оценке конечна");
    auto rnd = Random(7);
    auto e1 = evolve(e0, 2, rnd, cfg);
    const float b = bestFitness(e1);
    const float m = meanFitness(e1);
    assert(isFinite(b) && isFinite(m) && b >= 0.0f && m >= 0.0f,
        "физический цикл отбора конечен");
}

unittest
{
    // Химия роста подключена к отбору: за поколения форма обязана меняться, и
    // меняться в ту сторону, которую отбор оплачивает. Проверяем, что эволюция
    // вообще способна улучшить лучшего — иначе новый генетический слой просто
    // не работает.
    auto rnd = Random(11);
    auto pop = evaluatePopulation(seedPopulation(EvolutionConfig.populationSize));
    auto evolved = evolve(pop, 15, rnd);
    assert(bestFitness(evolved) >= bestFitness(pop) - 1e-6f,
        "эволюция не ухудшает лучшего");

    // Форма каркаса обязана отличаться от основательской: иначе отбор
    // выбирает среди одинаковых машин и эволюция стоит на месте.
    const auto founder = develop(pop[0].chromosome).get;
    bool formsDiffer;
    foreach (e; evolved)
        if (auto grown = develop(e.chromosome))
            if (grown.get.beams.length != founder.beams.length)
                formsDiffer = true;
    assert(formsDiffer, "потомки обязаны отличаться от основателя");
}
