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
import viewer.terrainvisualizer;

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
    Texture texBeam;
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

    // Живой заезд realtime:
    private BuggyPhysics livePhysics;
    private Entity[] liveCar;
    private double liveSimTime;
    private double liveRunSeconds;

    /// Камера-орбита; в живом заезде плавно ведёт центр масс машины.
    private FreeviewComponent freeview;
    private bool liveFailed_;
    private bool nWasHandled_;

    /// Визуализатор процедурной поверхности (общий shared-кэш с фитнесом).
    private TerrainVisualizer terrainVis;

    private Buggy[] liveBatch;
    private size_t liveBatchIdx;

    // Гонка поколений: строится на главном потоке, физика считается в фоне.
    private Individual[] cur;
    private size_t runGens;
    private size_t gensDone;
    private size_t jobGen;
    private PhysicsBatch batch;
    private EvolutionConfig runCfg;

    override void afterLoad()
    {
        // BuggyScene создан через New! (dlib): GC не сканирует dlib-память.
        // Регистрируем объект сцены как GC-диапазон, чтобы ссылки в его полях
        // (grammar, population, terrainVis) не уносились сборщиком.
        // afterLoad, как и update, может быть вызван позже при загрузке сцены.
        import core.memory: GC;
        GC.addRange(cast(void*)this, __traits(classInstanceSize, BuggyScene));

        eventManager.trackUpDownState = true;
        grammar = buggyGrammar();
        rnd = Random(42);
        // Гибридная оценка: статический гейт + 3 секунды физического заезда.
        evolutionConfig.simulateSeconds = 3.0;
        liveRunSeconds = evolutionConfig.simulateSeconds;
        // Виден ли ход заездов в stdout (по особам и поколениям).
        evolutionConfig.logPhysics = true;

        auto camera = addCamera();
        freeview = New!FreeviewComponent(eventManager, camera);
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
        matBeam.baseColorTexture = buildBeamGradientTexture();

        matWheel = addMaterial();
        matWheel.baseColorFactor = Color4f(0.08f, 0.08f, 0.08f, 1.0f);
        matWheel.roughnessFactor = 0.9f;
        matWheel.metallicFactor = 0.0f;

        matDriveWheel = addMaterial();
        matDriveWheel.baseColorFactor = Color4f(0.6f, 0.1f, 0.1f, 1.0f);
        matDriveWheel.roughnessFactor = 0.9f;
        matDriveWheel.metallicFactor = 0.0f;

        // Плоская «земля» больше не нужна: её рисует процедурная поверхность
        // TerrainVisualizer (общий shared-кэш с фитнесом).
        terrainVis = new TerrainVisualizer(this, sharedTerrain());

        resetPopulation();
        buildGallery();
        logGeneration();
    }

    /// Текстура-градиент для балок вдоль их длины. Цилиндр балки ориентирован
    /// так, что его верхний торец (v=0) лежит у узла b.b («конечного» конца),
    /// а нижний (v=1) — у узла b.a («начального»). Поэтому v=0 получает текущий
    /// цвет балки, а к v=1 цвет плавно темнеет.
    private Texture buildBeamGradientTexture()
    {
        const int imgW = 4;
        const int imgH = 64;
        const Color4f dark = Color4f(0.19f, 0.19f, 0.21f, 1.0f);
        const Color4f light = matBeam.baseColorFactor;

        SuperImage img = unmanagedImage(imgW, imgH, 4, 8);
        foreach (y; 0 .. imgH)
        {
            const float t = cast(float)y / cast(float)(imgH - 1);
            const Color4f c = Color4f(
                light.r + (dark.r - light.r) * t,
                light.g + (dark.g - light.g) * t,
                light.b + (dark.b - light.b) * t,
                1.0f);
            foreach (x; 0 .. imgW)
                img[x, y] = c;
        }

        auto tex = New!Texture(this);
        tex.createFromImage(img, false);
        Delete(img);
        return tex;
    }

    /// Новое 0-е поколение: идентичные копии закодированного багги.
    private void resetPopulation()
    {
        population = evaluatePopulation(grammar, seedPopulation(grammar, EvolutionConfig.populationSize));
        generation = 0;
    }

    override void update(Time t)
    {
        super.update(t);

        // Окно поверхности за машиной (или в origin, пока машин нет).
        // afterLoad может не завершиться к первому кадру — terrainVis ещё null.
        if (terrainVis !is null)
            terrainVis.update(livePhysics);

        // Камера-орбита плавно ведёт центр массы живой машины; угол обзора
        // остаётся за мышью (повороты/зум не сбрасываются). Точка орбиты в
        // FreeviewComponent инвертирована (см. targetEntity: target = -pos),
        // поэтому передаём позицию с минусом.
        if (freeview !is null && livePhysics !is null)
            freeview.setTargetSmooth(-Vector3f(livePhysics.worldFocus));

        if (jobThread !is null)
        {
            if (atomicLoad(jobDone))
            {
                jobThread = null;
                cur = batch.res;
                generation++;
                if (gensDone + 1 < runGens)
                {
                    gensDone++;
                    startNextGen();
                }
                else
                {
                    population = cur;
                    stopLiveCar();
                    buildGallery();
                    logGeneration();
                }
            }
            else
                stepLiveCar();

            // Переключение симулируемой особи — только вручную, по N.
            // keyPressed — защёлка события; игнорируем повторы, пока клавиша
            // не отпущена (одно переключение на одно нажатие).
            const bool nHeld = eventManager.keyPressed[KEY_N];
            if (nHeld && !nWasHandled_ && livePhysics !is null)
            {
                nWasHandled_ = true;
                if (showNextLiveBuggy())
                    updateLiveCar();
            }
            if (!nHeld)
                nWasHandled_ = false;
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
    }

    private void submitEvolution(size_t generations, EvolutionConfig cfg)
    {
        // Сброс до старта: иначе устаревший true от прошлого задания
        // мгновенно применяет результат и гасит live-заезд.
        atomicStore(jobDone, false);
        runCfg = cfg;
        cur = population;
        runGens = generations;
        gensDone = 0;
        jobThread = null;
        startNextGen();
        removeCar();
        startLiveCar();
    }

    /// Строит партию поколения на главном потоке и запускает её физику в фоне.
    private void startNextGen()
    {
        auto children = buildNextGeneration(grammar, cur, runCfg, rnd);
        batch = evaluateStatic(grammar, children, runCfg);
        jobGen = generation + 1;
        atomicStore(jobDone, false);
        jobThread = new Thread({
            runPhysics(batch, runCfg, jobGen);
            atomicStore(jobDone, true);
        });
        jobThread.isDaemon = true;
        jobThread.start();
    }

    private void startLiveCar()
    {
        liveBatch = null;
        liveBatchIdx = 0;
    }

    private bool showNextLiveBuggy()
    {
        reload:
        if (liveBatch is null || liveBatchIdx >= liveBatch.length)
        {
            auto next = batch.needPhysics;
            if (next is liveBatch || next.length == 0)
                return false;
            liveBatch = next;
            liveBatchIdx = 0;
        }

        foreach (attempt; liveBatchIdx .. liveBatch.length)
        {
            Frame frame = liveBatch[attempt].frame;
            if (frame.anchors.length < 2 || !canDrive(frame))
                continue;

            // Живой заезд — по той же процедурной поверхности, что и фитнес.
            auto physics = new BuggyPhysics(new Buggy(frame, origin),
                sharedTerrain());
            physics.settle(physicsDt,
                cast(int)(physicsSettleSeconds / physicsDt));
            const settleFailure = runFailure(physics);
            if (settleFailure.length)
            {
                writefln("live: заезд оборван после усадки (%s) — следующая машина",
                    settleFailure);
                physics.dispose();
                continue;
            }

            if (livePhysics !is null)
            {
                livePhysics.dispose();
                livePhysics = null;
            }
            removeLiveEntities();

            liveBatchIdx = attempt + 1;
            livePhysics = physics;
            liveSimTime = 0.0;
            liveFailed_ = false;

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
            return true;
        }

        liveBatchIdx = liveBatch.length;
        goto reload;
    }

    private void stepLiveCar()
    {
        if (livePhysics is null)
        {
            if (showNextLiveBuggy())
                updateLiveCar();
            return;
        }

        livePhysics.step(physicsDt, 1.0f);
        liveSimTime += physicsDt;

        const stepFailure = runFailure(livePhysics);
        if (stepFailure.length && !liveFailed_)
        {
            liveFailed_ = true;
            writefln("live: заезд оборван (%s) — ждём клавиши N", stepFailure);
        }

        updateLiveCar();
    }

    private void removeLiveEntities()
    {
        foreach (e; liveCar)
        {
            removeEntity(e);
            carRoot.removeChild(e);
        }
        liveCar.length = 0;
    }

    private void updateLiveCar()
    {
        if (livePhysics is null)
            return;

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
        removeLiveEntities();
        if (livePhysics !is null)
        {
            livePhysics.dispose();
            livePhysics = null;
        }
        liveBatch = null;
        liveBatchIdx = 0;
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
        vec3 c = origin;
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