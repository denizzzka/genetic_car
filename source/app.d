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
import dlib.math.utils : clamp;
version(linux)
{
    import etc.linux.memoryerror;
    version(DigitalMars)
    {
        shared static this()
        {
            registerMemoryAssertHandler();
        }
    }
}

/// Headless-прогон: тот же отбор, что и по G во вьюере, но без окна —
/// эволюция стартует сразу и печатает поколения в stdout.
private struct HeadlessOptions
{
    size_t generations = 400;
    size_t seed = 42;
    double simulateSeconds = physicsSimSeconds;
    size_t threads = 0;
    bool logPhysics = false;
    bool logStats = false;
}

private HeadlessOptions parseHeadless(string[] args)
{
    HeadlessOptions opt;
    for (size_t i = 0; i < args.length; ++i)
    {
        const auto arg = args[i];
        if (arg.length < 2 || arg[0] != '-')
            continue;
        switch (arg)
        {
            case "-gens", "--gens":
                if (i + 1 < args.length)
                    opt.generations = to!size_t(args[++i]);
                break;
            case "-seed", "--seed":
                if (i + 1 < args.length)
                    opt.seed = to!size_t(args[++i]);
                break;
            case "-threads", "--threads":
                if (i + 1 < args.length)
                    opt.threads = to!size_t(args[++i]);
                break;
            case "-seconds", "--seconds":
                if (i + 1 < args.length)
                    opt.simulateSeconds = to!double(args[++i]);
                break;
            case "-log", "--log":
                opt.logPhysics = true;
                break;
            case "-stats", "--stats":
                opt.logStats = true;
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
    cfg.logPhysics = opt.logPhysics;
    cfg.logStats = opt.logStats;

    if (opt.threads > 0)
        setWorkPoolSize(opt.threads);

    writefln("headless: engine %s, %d поколений, зерно %d, окно заезда %.1fs%s",
        selectedEngineName, opt.generations, opt.seed, opt.simulateSeconds,
        opt.logPhysics ? ", лог по особям" : "");

    auto cur = evaluatePopulation(seedPopulation(EvolutionConfig.populationSize), cfg);
    auto rnd = Random(cast(uint) opt.seed);

    StopWatch sw = StopWatch(AutoStart.yes);
    size_t prev = sw.peek.total!"msecs";
    foreach (gen; 0 .. opt.generations)
    {
        GenerationStats stats;
        auto nextGen = buildNextGeneration(cur, cfg, rnd,
            cfg.logStats ? &stats : null);
        const size_t growMs = sw.peek.total!"msecs";
        auto batch = evaluateStatic(nextGen, cfg);
        const size_t evalMs = sw.peek.total!"msecs";

        runPhysics(batch, cfg, gen + 1);
        cur = batch.res;
        const size_t now = sw.peek.total!"msecs";
        const GenerationTiming timing = GenerationTiming((growMs - prev) / 1000.0,
            (evalMs - growMs) / 1000.0, (now - evalMs) / 1000.0);
        if (cfg.logStats)
            stats.addPopulation(cur);
        writefln("поколение %d: needPhysics=%d best=%.4f mean=%.4f %s всего=%.1fs%s",
            gen + 1, batch.needPhysics.length, bestFitness(cur), meanFitness(cur),
            timing.toPhases(), now / 1000.0,
            cfg.logStats ? " | " ~ stats.line() : "");
        prev = now;
    }
    writefln("headless done: %d поколений, seed %d", opt.generations, opt.seed);
}

void main(string[] args)
{
    args = selectEngine(args);
    if (args.canFind("-probe"))
    {
        import physics_world.engineselect : defaultEngine, PhysicsEngine;
        import physics_world.worldpool : acquireWorld, releaseWorld;
        import frame.frame : Frame, Node, Beam, Anchor, AnchorKind,
            initialMotorPower, origin, frameUp = up, frameForward = forward,
            frameRight = right;
        import physics_world.physics : RunOutcome, steerNodeIndex;
        import frame.frame : heightOf;
        import physics_world.engineselect : selectedEngineName;
        import dlib.math.vector : vec3;
        Frame fork()
        {
            Frame frame;
            size_t node(vec3 pos)
            {
                frame.nodes ~= Node(pos);
                return frame.nodes.length - 1;
            }
            const c = node(vec3(0.0f, 0.3f, 0.35f));
            const nose = node(vec3(0.0f, -0.9f, 0.85f));
            const hub = node(vec3(0.0f, -1.0f, 0.5f));
            const fl = node(vec3(0.7f, -0.55f, 0.35f));
            const fr = node(vec3(-0.7f, -0.55f, 0.35f));
            const rl = node(vec3(0.7f, 0.4f, 0.35f));
            const rr = node(vec3(-0.7f, 0.4f, 0.35f));
            const nl = node(vec3(0.75f, -1.6f, 0.45f));
            const nr = node(vec3(-0.75f, -1.6f, 0.45f));
            frame.beams ~= new Beam(fl, fr, 0.045f);
            frame.beams ~= new Beam(rl, rr, 0.05f);
            frame.beams ~= new Beam(c, fl, 0.045f);
            frame.beams ~= new Beam(c, fr, 0.045f);
            frame.beams ~= new Beam(c, rl, 0.05f);
            frame.beams ~= new Beam(c, rr, 0.05f);
            frame.beams ~= new Beam(c, nose, 0.045f);
            frame.beams ~= new Beam(nose, hub, 0.05f);
            frame.beams ~= new Beam(hub, nl, 0.05f);
            frame.beams ~= new Beam(hub, nr, 0.05f);
            frame.anchors ~= Anchor(fl, AnchorKind.wheel);
            frame.anchors ~= Anchor(fr, AnchorKind.wheel);
            frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
            frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
            frame.anchors ~= Anchor(nl, AnchorKind.wheel);
            frame.anchors ~= Anchor(nr, AnchorKind.wheel);
            frame.motorPower = initialMotorPower;
            return frame;
        }
        Frame box()
        {
            Frame frame;
            size_t node(vec3 pos)
            {
                frame.nodes ~= Node(pos);
                return frame.nodes.length - 1;
            }
            const c = node(vec3(0.0f, 0.3f, 0.35f));
            const fl = node(vec3(0.7f, -0.55f, 0.35f));
            const fr = node(vec3(-0.7f, -0.55f, 0.35f));
            const rl = node(vec3(0.7f, 0.4f, 0.35f));
            const rr = node(vec3(-0.7f, 0.4f, 0.35f));
            frame.beams ~= new Beam(fl, fr, 0.045f);
            frame.beams ~= new Beam(rl, rr, 0.05f);
            frame.beams ~= new Beam(c, fl, 0.045f);
            frame.beams ~= new Beam(c, fr, 0.045f);
            frame.beams ~= new Beam(c, rl, 0.05f);
            frame.beams ~= new Beam(c, rr, 0.05f);
            frame.anchors ~= Anchor(fl, AnchorKind.wheel);
            frame.anchors ~= Anchor(fr, AnchorKind.wheel);
            frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
            frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
            frame.motorPower = initialMotorPower;
            return frame;
        }
        foreach (engine; [PhysicsEngine.jolt, PhysicsEngine.newton])
        {
            defaultEngine(engine);
            auto p = new BuggyPhysics(new Buggy(placedFrame(fork())));
            scope (exit) p.dispose();
            writefln("PROBE %s: isSteered=%s yaw0=%.3f",
                selectedEngineName, p.isSteered, p.steerYaw);
            const double dt = 1.0 / p.updateRate;
            p.autoSteer = false;
            foreach (i; 0 .. 90)
            {
                p.step(dt, 0.0f);
                if ((i & 7) == 0)
                    writefln("PROBE %s: settle t=%.2f yaw=%.3f",
                        selectedEngineName, i * dt, p.steerYaw);
            }
            writefln("PROBE %s: settled yaw=%.3f",
                selectedEngineName, p.steerYaw);
            p.steerJoint.targetYaw = 0.35f;
            foreach (i; 0 .. 180)
            {
                p.step(dt, 1.0f);
                if ((i & 15) == 0)
                    writefln("PROBE %s: t=%.2f yaw=%.3f fail=%s cab=%.3f",
                        selectedEngineName, i * dt, p.steerYaw,
                        cast(string) runFailure(p),
                        heightOf(p.cockpitState.position));
            }
        }
        {
            Frame forkNoWheels()
            {
                Frame frame;
                size_t node(vec3 pos)
                {
                    frame.nodes ~= Node(pos);
                    return frame.nodes.length - 1;
                }
                const c = node(vec3(0.0f, 0.3f, 0.35f));
                const nose = node(vec3(0.0f, -0.9f, 0.85f));
                const hub = node(vec3(0.0f, -1.0f, 0.5f));
                const fl = node(vec3(0.7f, -0.55f, 0.35f));
                const fr = node(vec3(-0.7f, -0.55f, 0.35f));
                const rl = node(vec3(0.7f, 0.4f, 0.35f));
                const rr = node(vec3(-0.7f, 0.4f, 0.35f));
                const nl = node(vec3(0.75f, -1.6f, 0.45f));
                const nr = node(vec3(-0.75f, -1.6f, 0.45f));
                frame.beams ~= new Beam(fl, fr, 0.045f);
                frame.beams ~= new Beam(rl, rr, 0.05f);
                frame.beams ~= new Beam(c, fl, 0.045f);
                frame.beams ~= new Beam(c, fr, 0.045f);
                frame.beams ~= new Beam(c, rl, 0.05f);
                frame.beams ~= new Beam(c, rr, 0.05f);
                frame.beams ~= new Beam(c, nose, 0.045f);
                frame.beams ~= new Beam(nose, hub, 0.05f);
                frame.beams ~= new Beam(hub, nl, 0.05f);
                frame.beams ~= new Beam(hub, nr, 0.05f);
                frame.anchors ~= Anchor(fl, AnchorKind.wheel);
                frame.anchors ~= Anchor(fr, AnchorKind.wheel);
                frame.anchors ~= Anchor(rl, AnchorKind.motorWheel);
                frame.anchors ~= Anchor(rr, AnchorKind.motorWheel);
                frame.motorPower = initialMotorPower;
                return frame;
            }
            foreach (engine; [PhysicsEngine.jolt, PhysicsEngine.newton])
            {
                defaultEngine(engine);
                auto p = new BuggyPhysics(new Buggy(placedFrame(forkNoWheels())));
                scope (exit) p.dispose();
                const double dt = 1.0 / p.updateRate;
                foreach (i; 0 .. 90)
                {
                    p.step(dt, 0.0f);
                    if ((i & 15) == 0)
                        writefln("PROBENW %s: settle t=%.2f yaw=%.3f",
                            selectedEngineName, i * dt, p.steerYaw);
                }
                writefln("PROBENW %s: settled yaw=%.3f",
                    selectedEngineName, p.steerYaw);
            }
        }
        foreach (engine; [PhysicsEngine.jolt, PhysicsEngine.newton])
        {
            defaultEngine(engine);
            auto p = new BuggyPhysics(new Buggy(placedFrame(box())));
            scope (exit) p.dispose();
            writefln("PROBEBOX %s: isSteered=%s",
                selectedEngineName, p.isSteered);
            const double dt = 1.0 / p.updateRate;
            p.settle(dt, 90);
            writefln("PROBEBOX %s: settled cab=%.3f fail=%s",
                selectedEngineName,
                heightOf(p.cockpitState.position),
                cast(string) runFailure(p));
            foreach (i; 0 .. 180)
                p.step(dt, 1.0f);
            writefln("PROBEBOX %s: after drive cab=%.3f fail=%s",
                selectedEngineName,
                heightOf(p.cockpitState.position),
                cast(string) runFailure(p));
        }
        return;
    }
    if (args.canFind("-headless") || args.canFind("--headless"))
        defaultEngine(PhysicsEngine.newton);
    if (args.canFind("-viab"))
    {
        import genetics.chromosome : Chromosome, mutated;
        import genetics.growth : develop;
        import genetics.fitness : buggyFitness;
        import std.stdio : writefln;
        import std.random : Random;
        import std.algorithm : min;
        auto rnd = Random(42);
        const Chromosome founder;
        size_t alive, nullFrames, wheels0, wheels1, wheels2p;
        size_t[8] hist;
        foreach (_; 0 .. 20)
        {
            const auto trial = mutated(founder, 0.05f, rnd);
            const auto grown = develop(trial);
            if (grown.isNull) { ++nullFrames; continue; }
            const auto f = grown.get;
            hist[min(f.anchors.length, size_t(7))]++;
            if (buggyFitness(f) > 0.0f) { ++alive; continue; }
            if (f.anchors.length == 0) ++wheels0;
            else if (f.anchors.length == 1) ++wheels1;
            else ++wheels2p;
        }
        writefln("VIAB alive=%d null=%d a0=%d a1=%d a2+=%d", alive, nullFrames, wheels0, wheels1, wheels2p);
        foreach (i, h; hist) writefln("   anchors=%d : %d", i, h);
        return;
    }
    if (args.canFind("-founder"))
    {
        import genetics.founder_tuning : FounderProbe, report, scanFounder,
            bestGyroscooter;
        auto probes = scanFounder();
        FounderProbe best;
        if (bestGyroscooter(probes, best))
            writefln("лучший гироскутер: %s", report(best));
        else
            writefln("гироскутерных проб нет");
        return;
    }
    if (args.canFind("-diag"))
    {
        import genetics.chromosome : Chromosome, mutated;
        import genetics.growth : develop;
        import genetics.fitness : buggyFitness;
        import frame.frame : AnchorKind;
        import dlib.math.vector : distance;
        import std.algorithm : min, max;
        import std.stdio : writefln;
        const Chromosome founder;
        const auto grown = develop(founder);
        if (grown.isNull) { writefln("DIAG founder null"); return; }
        const auto f = grown.get;
        size_t wi;
        foreach (a; f.anchors)
        {
            const vec3 c = f.nodes[a.node].pos;
            float best = float.max;
            size_t bestBeam;
            foreach (bi, b; f.beams)
            {
                const vec3 pa = f.nodes[b.a].pos, pb = f.nodes[b.b].pos;
                if (b.a == a.node || b.b == a.node)
                    continue;
                const vec3 ab = pb - pa;
                const float t = clamp(dot(c - pa, ab) / max(dot(ab, ab), 1e-9f), 0.0f, 1.0f);
                const float d = distance(c, pa + ab * t);
                if (d < best) { best = d; bestBeam = bi; }
            }
            writefln("DIAG wheel %d node=%d c=(%.2f,%.2f,%.2f) r=%.2f kind=%s ближайшая балка %d d=%.4f (порог %.4f) всего балок %d",
                wi, a.node, c.x, c.y, c.z, a.radius, a.kind, bestBeam, best,
                a.radius + founder.beamRadius, f.beams.length);
            ++wi;
        }
        float lo = float.max, hi = -float.max, sum = 0.0f;
        foreach (bi, b; f.beams)
        {
            const float len = distance(f.nodes[b.a].pos, f.nodes[b.b].pos);
            lo = min(lo, len);
            hi = max(hi, len);
            sum += len;
        }
        writefln("DIAG балки=%d длина min=%.3f avg=%.3f max=%.3f (шаг=%.3f)",
            cast(int) f.beams.length, lo, sum / f.beams.length, hi,
            founder.segments[0].stepLength);
        float nodeLo = float.max, nodeHi = -float.max, wheelLo = float.max;
        foreach (n; f.nodes)
        {
            nodeLo = min(nodeLo, n.pos.z);
            nodeHi = max(nodeHi, n.pos.z);
        }
        foreach (a; f.anchors)
            wheelLo = min(wheelLo, f.nodes[a.node].pos.z - a.radius);
        writefln("DIAG узлы z=%.2f..%.2f низ колеса=%.2f фитнес=%.4f",
            nodeLo, nodeHi, wheelLo, buggyFitness(f));
        import physics_world : Buggy, BuggyPhysics, placedFrame;
        import physics_world.wheel : steerLimitRad;
        auto bp = new BuggyPhysics(new Buggy(placedFrame(f)));
        scope (exit) bp.dispose();
        writefln("DIAG isSteered=%s steerYaw=%.3f", bp.isSteered, bp.steerYaw);
        bp.settle(1.0 / 60.0, 60);
        bp.autoSteer = false;
        bp.steerJoint.targetYaw = 8.0f;
        bp.settle(1.0 / 60.0, 360);
        writefln("DIAG steered yaw=%.4f (limit=%.4f)", bp.steerYaw, steerLimitRad);
        return;
    }
    if (args.canFind("-headless") || args.canFind("--headless"))
    {
        runHeadless(parseHeadless(args));
        return;
    }

    MyGame game = New!MyGame(1280, 720, false, "Genetic Car - Frame Viewer", args);
    game.run();
    Delete(game);
}
