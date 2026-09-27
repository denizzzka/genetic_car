module app;

import dagon;
import viewer;
import physics_world;

import genetics;
import std.algorithm: canFind;
import std.datetime.stopwatch: StopWatch, AutoStart;
import std.random : Random;
import std.conv : to;
import std.stdio;

private struct HeadlessOptions
{
    size_t generations = 400;
    size_t seed = 42;
    double simulateSeconds = 120.0;
    size_t threads = 0;
}

private HeadlessOptions parseHeadless(string[] args)
{
    HeadlessOptions opt;
    foreach (i, arg; args)
    {
        if (i + 1 >= args.length)
            break;
        switch (arg)
        {
            case "-gens", "--gens":
                opt.generations = to!size_t(args[i + 1]);
                ++i;
                break;
            case "-seed", "--seed":
                opt.seed = to!size_t(args[i + 1]);
                ++i;
                break;
            case "-threads", "--threads":
                opt.threads = to!size_t(args[i + 1]);
                ++i;
                break;
            case "-seconds", "--seconds":
                opt.simulateSeconds = to!double(args[i + 1]);
                ++i;
                break;
            default:
                break;
        }
    }
    return opt;
}

private void runHeadless(const HeadlessOptions opt)
{
    EvolutionConfig cfg = EvolutionConfig.init;
    cfg.simulateSeconds = opt.simulateSeconds;

    if (opt.threads > 0)
        setPhysicsPoolSize(opt.threads);

    auto gr = buggyGrammar();
    auto cur = evaluatePopulation(gr, seedPopulation(gr, EvolutionConfig.populationSize), cfg);
    auto rnd = Random(cast(uint) opt.seed);

    StopWatch sw = StopWatch(AutoStart.yes);
    size_t prev = sw.peek.total!"msecs";
    foreach (gen; 0 .. opt.generations)
    {
        auto batch = evaluateStatic(gr, buildNextGeneration(gr, cur, cfg, rnd), cfg);
        runPhysics(batch, cfg, gen + 1);
        cur = batch.res;
        const size_t now = sw.peek.total!"msecs";
        writefln("gen %d: needPhysics=%d best=%.4f dt=%.2fs total=%.1fs",
            gen + 1, batch.needPhysics.length, bestFitness(cur),
            (now - prev) / 1000.0, now / 1000.0);
        prev = now;
    }
    writefln("headless done: %d generations, seed %d", opt.generations, opt.seed);
}

void main(string[] args)
{
    if (args.canFind("-headless") || args.canFind("--headless"))
    {
        runHeadless(parseHeadless(args));
        return;
    }

    MyGame game = New!MyGame(1280, 720, false, "Genetic Car - Frame Viewer", args);
    game.run();
    Delete(game);
}
