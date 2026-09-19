module viewer.viewer;

import dagon;
import dagon.core.keycodes;
import dagon.core.time;
import core.thread : Thread;
import core.sync : Mutex, Condition;
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

    // ---- Фоновая эволюция ----
    // Физическая оценка поколения идёт в отдельном потоке: основной
    // (rnd/отрисовка) не блокируется и работает как приложение. Всё
    // перекрёстное состояние закрыто мьютексом; результат применяется
    // к сцене только в update (главный поток) — dagon-сущности из
    // воркера не трогаются.
    private Thread worker;
    // Создаются в afterLoad: new Condition/pthread не терпит CTFE-инициализации.
    private Mutex evolMutex;
    private Condition evolCond;
    private bool workerBusy;     // поданная работа ещё не применена к сцене
    private bool jobRequested;   // воркеру поставлена задача
    private bool jobDone;        // воркер закончил, результат ждёт применения
    private bool stopWorker;     // останова воркера при завершении
    private Individual[] jobInput;
    private size_t jobGenerations;
    private EvolutionConfig jobConfig;
    private Individual[] jobOutput;

    // ---- Живой заезд (только главный поток) ----
    // Пока поколение считается в фоне, на витрине едет одна машина в
    // реальном времени — BuggyPhysics шагает раз за update(~1/60 с).
    // Сущности под carRoot: liveCar[0] — рама, далее колёса по anchors.
    private BuggyPhysics livePhysics;
    private Entity[] liveCar;
    private Mesh meshChassis = null;
    private Vector3f chassisScale;
    private Frame liveFrame;
    private double liveSimTime;

    /// Длительность показанного «круга» в реальном времени: как физический
    /// заезд особи (evolutionConfig.simulateSeconds), после — повторить,
    /// иначе машина уехала бы за пределы витрины на долгом прогоне.
    enum double liveRunSeconds = 3.0;

    override void afterLoad()
    {
        eventManager.trackUpDownState = true;
        grammar = buggyGrammar();
        rnd = Random(42);
        // Гибридная оценка: статический гейт + 3 секунды физического заезда.
        evolutionConfig.simulateSeconds = 3.0;
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
        // Полуразмерный бокс (1×1×1): масштабом e.scaling = габариты рамы.
        meshChassis = New!ShapeBox(Vector3f(0.5f, 0.5f, 0.5f), assetManager);

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

        evolMutex = new Mutex;
        evolCond = new Condition(evolMutex);
        seedWorker();

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

        if (workerBusy)
        {
            // Фон считает поколение: применяем результат, когда готов.
            evolMutex.lock();
            const pending = jobDone;
            evolMutex.unlock();

            if (pending)
            {
                evolMutex.lock();
                auto fresh = jobOutput;
                const gens = jobGenerations;
                jobOutput = null;
                jobDone = false;
                evolMutex.unlock();

                population = fresh;
                generation += gens;
                workerBusy = false;
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
        {
            submitEvolution(evolutionConfig.generationsPerPress, evolutionConfig);
        }
        else if (eventManager.keyDown[KEY_M])
        {
            // Быстрый шаг: только статический отбор, без физического заезда.
            submitEvolution(1, EvolutionConfig.init);
        }
    }

    /// Поднять фоновый воркер эволюции. Демон: при выходе из приложения
    /// процесс завершается, не дожидаясь потока.
    private void seedWorker()
    {
        worker = new Thread(&workerRun);
        worker.isDaemon = true;
        worker.start();
    }

    /// Цикл воркера: ждёт задачу, считает поколения в физическом пуле,
    /// кладёт результат под мьютекс. `rnd` свой (зерно фиксировано —
    /// отбор воспроизводим в рамках сессии, как раньше на главном потоке).
    private void workerRun()
    {
        auto workerRnd = Random(42);
        while (true)
        {
            evolMutex.lock();
            while (!jobRequested && !stopWorker)
                evolCond.wait();
            if (stopWorker)
            {
                evolMutex.unlock();
                return;
            }
            jobRequested = false;
            auto input = jobInput;
            const gens = jobGenerations;
            const cfg = jobConfig;
            evolMutex.unlock();

            auto output = evolve(grammar, input, gens, workerRnd, cfg);

            evolMutex.lock();
            jobOutput = output;
            jobDone = true;
            evolCond.notify();
            evolMutex.unlock();
        }
    }

    /// Поставить задачу фоновой эволюции и запустить живой заезд.
    private void submitEvolution(size_t generations, EvolutionConfig cfg)
    {
        evolMutex.lock();
        jobInput = population;
        jobGenerations = generations;
        jobConfig = cfg;
        jobOutput = null;
        jobDone = false;
        jobRequested = true;
        evolCond.notify();
        evolMutex.unlock();

        workerBusy = true;
        // На время заезда убираем стоящую витрину топ-5: на сцене остаётся
        // только едущий потомок победителя. Галерея вернётся в buildGallery
        // по завершении работы.
        removeCar();
        startLiveCar();
    }

    /// Пока считается поколение, на витрине едет потомок лучшего багги
    /// предыдущего поколения: кроссовер победителя с партнёром + мутация —
    /// как buildNextGeneration, только на одном геноме.
    private void startLiveCar()
    {
        auto descendant = descendantOfBest(grammar, population,
            evolutionConfig.tournamentSize, evolutionConfig.mutateHits, rnd);
        auto may = develop(grammar, descendant);
        if (may.isNull)
            return;
        auto frame = may.get.frame;
        if (frame.anchors.length < 2)
            return;

        livePhysics = new BuggyPhysics(new Buggy(frame, vec3(0.0f)));
        livePhysics.setSlopeDeg(physicsSlopeDeg);
        livePhysics.settle(physicsDt,
            cast(int)(physicsSettleSeconds / physicsDt));
        liveFrame = frame;
        liveSimTime = 0.0;

        // Габариты рамы под кузов-бокс: как в BuggyPhysics.buildChassis.
        vec3 minP = vec3(float.max, float.max, float.max);
        vec3 maxP = vec3(-float.max, -float.max, -float.max);
        foreach (n; frame.nodes)
        {
            minP.x = min(minP.x, n.pos.x);
            minP.y = min(minP.y, n.pos.y);
            minP.z = min(minP.z, n.pos.z);
            maxP.x = max(maxP.x, n.pos.x);
            maxP.y = max(maxP.y, n.pos.y);
            maxP.z = max(maxP.z, n.pos.z);
        }
        vec3 dims = maxP - minP;
        dims.x = max(dims.x, 0.1f);
        dims.y = max(dims.y, 0.1f);
        dims.z = max(dims.z, 0.1f);
        // Небольшой запас по габариту — кузов зрительно обнимает раму.
        chassisScale = dims + vec3(0.1f, 0.1f, 0.1f);

        liveCar ~= addEntity(carRoot); // рама
        liveCar[$ - 1].drawable = meshChassis;
        liveCar[$ - 1].material = matBeam;
        liveCar[$ - 1].scaling = chassisScale;

        foreach (a; frame.anchors) // колёса
        {
            auto e = addEntity(carRoot);
            e.drawable = meshWheel;
            e.material = a.kind == AnchorKind.motorWheel ? matDriveWheel : matWheel;
            liveCar ~= e;
        }
    }

    /// Шаг живого заезда раз в update и перенос состояний в сущности.
    private void stepLiveCar()
    {
        if (livePhysics is null)
            return;
        livePhysics.step(physicsDt, 0.0f);
        liveSimTime += physicsDt;
        if (liveSimTime >= liveRunSeconds)
            restartLiveCar();
        else
            updateLiveCar();
    }

    /// Новый круг того же багги, чтобы машина не уезжала со сцены.
    private void restartLiveCar()
    {
        if (livePhysics !is null)
            livePhysics.dispose();
        livePhysics = new BuggyPhysics(new Buggy(liveFrame, vec3(0.0f)));
        livePhysics.setSlopeDeg(physicsSlopeDeg);
        livePhysics.settle(physicsDt,
            cast(int)(physicsSettleSeconds / physicsDt));
        liveSimTime = 0.0;
        updateLiveCar();
    }

    /// Перенос состояний физики в dagon-сущности живого багги.
    private void updateLiveCar()
    {
        const beams = livePhysics.beamStates();
        if (beams.length > 0 && liveCar.length > 0)
        {
            liveCar[0].position = beams[0].position;
            liveCar[0].rotation = beams[0].orientation;
        }

        const wheels = livePhysics.wheelStates();
        foreach (i; 1 .. liveCar.length)
            if (i <= wheels.length)
            {
                liveCar[i].position = wheels[i - 1].position;
                liveCar[i].rotation = wheels[i - 1].orientation;
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