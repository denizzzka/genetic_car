module viewer.viewer;

import dagon;
import dagon.core.keycodes;
import dagon.core.time;
import core.thread : Thread;
import core.atomic : atomicStore, atomicLoad;
import std.algorithm : min, max, sort;
import std.random;
import std.stdio : writefln;
import frame.frame;
import genetics;
import physics_world;

class BuggyScene: Scene
{
    MyGame game;

    this(MyGame game)
    {
        super(game);
        this.game = game;
    }

    override void beforeLoad()
    {
    }

    Entity carRoot;

    /// Геометрия и материалы, переиспользуемые между поколениями:
    /// создаются один раз, чтобы перестройка галереи не накапливала
    /// меши и материалы (dlib-память вне GC, иначе — утечка и крах).
    Mesh meshBeam = null;
    Mesh meshWheel = null;
    Material matBeam;
    Material matWheel;
    Material matDriveWheel;

    Grammar grammar;
    Random rnd;

    /// Текущая популяция отбора и номер поколения.
    Individual[] population;
    size_t generation;

    EvolutionConfig evolutionConfig;

    enum size_t galleryTop = 5;
    enum float gallerySpacing = 4.0f;

    /// Фоновый поток при создании новых поколений
    private Thread jobThread;
    private shared bool jobDone;
    private Individual[] jobOutput;
    private size_t jobGens;

    // Живой заезд realtime:
    private BuggyPhysics livePhysics;
    private Entity[] liveCar;
    private Frame liveFrame;
    private double liveSimTime;
    // Круг витрины = длительность заезда особи (simulateSeconds из конфига
    // оценки); после — заново, иначе машина уедет за сцену.
    private double liveRunSeconds;

    override void afterLoad()
    {
        eventManager.trackUpDownState = true;
        grammar = buggyGrammar();
        rnd = Random(42);
        // Гибридная оценка: статический гейт + 3 секунды физического заезда.
        evolutionConfig.simulateSeconds = 3.0;
        liveRunSeconds = evolutionConfig.simulateSeconds;
        // Виден ли ход заездов в stdout (по особам и поколениям).
        evolutionConfig.logPhysics = true;

        auto camera = addCamera();
        auto freeview = New!FreeviewComponent(eventManager, camera);
        freeview.setZoom(15.0f);
        freeview.setRotation(30.0f, -45.0f, 0.0f);
        freeview.translationStiffness = 0.25f;
        freeview.rotationStiffness = 0.25f;
        freeview.zoomStiffness = 0.25f;
        game.renderer.activeCamera = camera;

        auto sun = addLight(LightType.Sun);
        sun.shadowEnabled = true;
        sun.energy = 10.0f;
        sun.pitch(-45.0f);

        carRoot = addEntity();
        carRoot.rotation = rotationQuaternion(Vector3f(1, 0, 0), degtorad(-90.0f));

        meshBeam = New!ShapeCylinder(1.0f, 1.0f, 8, assetManager);
        meshWheel = New!ShapeTorus(0.2f, 0.1f, 16, 8, assetManager);

        matBeam = addMaterial();
        matBeam.baseColorFactor = Color4f(0.55f, 0.55f, 0.62f, 1.0f);
        matBeam.metallicFactor = 0.7f;

        matWheel = addMaterial();
        matWheel.baseColorFactor = Color4f(0.08f, 0.08f, 0.08f, 1.0f);
        matWheel.roughnessFactor = 0.9f;
        matWheel.metallicFactor = 0.0f;

        matDriveWheel = addMaterial();
        matDriveWheel.baseColorFactor = Color4f(0.6f, 0.1f, 0.1f, 1.0f);
        matDriveWheel.roughnessFactor = 0.9f;
        matDriveWheel.metallicFactor = 0.0f;

        auto ePlane = addEntity();
        ePlane.drawable = New!ShapePlane(12.0f, 12.0f, 1, assetManager);

        resetPopulation();
        buildGallery();
        logGeneration();

        /*
        BuggyScene is dlib-allocated (New!), so the GC can't see
        references to objects (grammar, population) stored in its fields.
        After enough GC pressure, these objects get collected, and the
        next access SIGSEGVs.

        It is need to register the scene's memory as a GC range so
        the GC scans its fields for pointers.
        */
        import core.memory: GC;
        GC.addRange(cast(void*)this, __traits(classInstanceSize, BuggyScene));
    }

    /// Новое 0-е поколение: идентичные копии закодированного багги.
    private void resetPopulation()
    {
        population = evaluatePopulation(grammar, seedPopulation(grammar, evolutionConfig.populationSize));
        generation = 0;
    }

    override void update(Time t)
    {
        super.update(t);

        if (jobThread !is null)
        {
            if (atomicLoad(jobDone))
            {
                population = jobOutput;
                generation += jobGens;
                jobThread = null;
                stopLiveCar();
                buildGallery();
                logGeneration();
            }
            else
                stepLiveCar();
            return;
        }

        if (eventManager.keyDown[KEY_R])
        {
            resetPopulation();
            buildGallery();
            logGeneration();
        }
        else if (eventManager.keyDown[KEY_G])
            submitEvolution(evolutionConfig.generationsPerPress, evolutionConfig);
        else if (eventManager.keyDown[KEY_M])
            submitEvolution(1, EvolutionConfig.init);
    }

    private void submitEvolution(size_t generations, EvolutionConfig cfg)
    {
        const gr = grammar;
        auto input = population;
        // Сброс до старта: иначе устаревший true от прошлого задания
        // мгновенно применяет результат и гасит live-заезд.
        atomicStore(jobDone, false);
        jobThread = new Thread({
            auto rnd = Random(42);
            jobOutput = evolve(gr, input, generations, rnd, cfg);
            jobGens = generations;
            atomicStore(jobDone, true);
        });
        jobThread.isDaemon = true;
        jobThread.start();
        removeCar();
        startLiveCar();
    }

    private void startLiveCar()
    {
        // Живой образ потомка лучшего. Сразу после усадки и на каждом шаге
        // применяется общий с фитнесом вердикт (runFailure): оборванный заезд
        // показывает разбитую машину, поэтому берём нового потомка.
        const int maxAttempts = 8;
        foreach (_; 0 .. maxAttempts)
        {
            auto descendant = descendantOfBest(grammar, population,
                evolutionConfig.tournamentSize, evolutionConfig.mutateHits, rnd);
            auto may = develop(grammar, descendant);
            if (may.isNull)
                continue;
            auto frame = may.get.frame;
            if (frame.anchors.length < 2)
                continue;

            auto physics = new BuggyPhysics(new Buggy(frame, vec3(0.0f)));
            physics.settle(physicsDt,
                cast(int)(physicsSettleSeconds / physicsDt));
            const settleFailure = runFailure(physics);
            if (settleFailure.length)
            {
                writefln("live: заезд оборван после усадки (%s) — другой потомок",
                    settleFailure);
                physics.dispose();
                continue;
            }

            livePhysics = physics;
            liveFrame = frame;
            liveSimTime = 0.0;

            // По одному цилиндру на каждую балку каркаса: порядок совпадает
            // с BeamState[] из beamStates() (по Frame.beams).
            foreach (b; frame.beams)
            {
                const float len =
                    (frame.nodes[b.b].pos - frame.nodes[b.a].pos).length;
                auto e = addEntity(carRoot);
                e.drawable = meshBeam;
                e.material = matBeam;
                e.scaling = Vector3f(b.radius, len, b.radius);
                liveCar ~= e;
            }

            foreach (a; frame.anchors)
            {
                auto e = addEntity(carRoot);
                e.drawable = meshWheel;
                e.material = a.kind == AnchorKind.motorWheel ? matDriveWheel : matWheel;
                liveCar ~= e;
            }
            return;
        }
        writefln("live: не удалось показать ни одного потомка");
    }

    private void stepLiveCar()
    {
        if (livePhysics is null)
            return;
        livePhysics.step(physicsDt, 1.0f);
        liveSimTime += physicsDt;
        const stepFailure = runFailure(livePhysics);
        if (stepFailure.length)
        {
            writefln("live: заезд оборван (%s) — другой потомок", stepFailure);
            stopLiveCar();
            startLiveCar();
            return;
        }
        if (liveSimTime >= liveRunSeconds)
            restartLiveCar();
        else
            updateLiveCar();
    }

    private void restartLiveCar()
    {
        if (livePhysics !is null)
            livePhysics.dispose();
        livePhysics = new BuggyPhysics(new Buggy(liveFrame, vec3(0.0f)));
        livePhysics.settle(physicsDt,
            cast(int)(physicsSettleSeconds / physicsDt));
        liveSimTime = 0.0;
        updateLiveCar();
    }

    private void updateLiveCar()
    {
        const beams = livePhysics.beamStates();
        foreach (i, s; beams)
            if (i < liveCar.length)
            {
                liveCar[i].position = s.position;
                liveCar[i].rotation = s.orientation;
            }

        const wheels = livePhysics.wheelStates();
        foreach (i, s; wheels)
        {
            const size_t idx = beams.length + i;
            if (idx < liveCar.length)
            {
                liveCar[idx].position = s.position;
                liveCar[idx].rotation = s.orientation;
            }
        }
    }

    private void stopLiveCar()
    {
        foreach (e; liveCar)
        {
            removeEntity(e);
            carRoot.removeChild(e);
        }
        liveCar.length = 0;
        if (livePhysics !is null)
        {
            livePhysics.dispose();
            livePhysics = null;
        }
    }

    private void logGeneration()
    {
        writefln("gen %d: best=%.4f mean=%.4f pop=%d",
            generation, bestFitness(population), meanFitness(population),
            population.length);
    }

    private void removeCar()
    {
        Entity[] toRemove;
        foreach (e; carRoot.children)
            toRemove ~= e;

        // Снимаем детей с корня и из мира: иначе сущности навечно
        // остаются в carRoot.children и с каждым поколением галерея
        // накапливает сотни сущностей в сцене.
        foreach (e; toRemove)
        {
            removeEntity(e);
            carRoot.removeChild(e);
        }
    }

    /// Витрина: топ-min(galleryTop) лучших в ряд по убыванию фитнеса.
    private void buildGallery()
    {
        removeCar();

        Individual[] ranked = new Individual[population.length];
        foreach (i, e; population)
            ranked[i] = e;
        sort!((a, b) => a.fitness > b.fitness)(ranked);

        const n = min(cast(size_t) galleryTop, ranked.length);
        const float firstX = (n - 1) * 0.5f * gallerySpacing;

        foreach (i; 0 .. n)
        {
            auto f = develop(grammar, ranked[i].genotype);
            if (f.isNull)
                continue;
            const float laneX = i * gallerySpacing - firstX;
            // Buggy — только каркас для отрисовки; физика строится отдельно
            // (BuggyPhysics) при оценке заезда. Офсет полосы — отображение,
            // не геометрия.
            auto buggy = new Buggy(f.get.frame, laneOffset(f.get.frame, laneX));
            drawBuggy(buggy);
        }
    }

    /// Рисует машину из Buggy: статичные балки и колёса в своей полосе laneX.
    private void drawBuggy(const Buggy buggy)
    {
        const off = buggy.offset;

        foreach (b; buggy.frame.beams)
        {
            const a = buggy.frame.nodes[b.a].pos + off;
            const b2 = buggy.frame.nodes[b.b].pos + off;
            const dir = b2 - a;
            const float length = dir.length;
            if (length < 1e-5f)
                continue;

            auto e = addEntity(carRoot);
            e.drawable = meshBeam;
            e.material = matBeam;
            e.position = (a + b2) * 0.5f;
            e.rotation = rotationBetween(Vector3f(0, 1, 0), dir / length);
            e.scaling = Vector3f(b.radius, length, b.radius);
        }

        foreach (anchor; buggy.frame.anchors)
        {
            const pos = buggy.frame.nodes[anchor.node].pos + off;
            final switch (anchor.kind)
            {
                case AnchorKind.wheel:
                    addWheel(pos, matWheel);
                    break;
                case AnchorKind.motorWheel:
                    addWheel(pos, matDriveWheel);
                    break;
            }
        }
    }

    private void addWheel(const vec3 pos, Material mat)
    {
        auto e = addEntity(carRoot);
        e.drawable = meshWheel;
        e.material = mat;
        e.position = pos;
        e.rotation = rotationBetween(Vector3f(0, 1, 0), Vector3f(1, 0, 0));
    }

    /// Центрует каркас горизонтально по среднему, ставит на землю и
    /// сдвигает в свою полосу вдоль X (в координатах машины).
    private vec3 laneOffset(const Frame f, float laneX)
    {
        vec3 c = vec3(0.0f);
        foreach (n; f.nodes)
            c += n.pos;
        if (f.nodes.length > 0)
            c /= f.nodes.length;

        float minZ = float.max;
        foreach (a; f.anchors)
            minZ = min(minZ, f.nodes[a.node].pos.z);

        float lift = 0.0f;
        if (minZ < float.max)
        {
            // 0.3 — радиус колеса (physics_world.wheelRadius). Всегда прижимаем
            // низ самого низкого колеса к земле — даже если эволюция унесла
            // каркас выше: иначе машины дрейфовали бы вверх и «улетали».
            lift = 0.3f - minZ;
        }

        return vec3(laneX - c.x, -c.y, lift);
    }
}

class MyGame: Game
{
    this(uint w, uint h, bool fullscreen, string title, string[] args)
    {
        super(w, h, fullscreen, title, args);
        currentScene = New!BuggyScene(this);
    }
}