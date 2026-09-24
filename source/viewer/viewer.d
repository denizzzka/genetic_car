module viewer.viewer;

import dagon;
import dagon.core.keycodes;
import dagon.core.time;
import core.thread : Thread;
import core.atomic : atomicStore, atomicLoad;
import std.algorithm : min, max, sort, map;
import std.array : array;
import std.random;
import std.stdio : writefln;
import frame.frame;
import frame.frame : frameForward = forward, frameUp = up;
import frame.cockpit : loadCockpit;
import frame.objmesh : ObjModel;
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

    /// Корень витрины: «топ-5» галереи живут отдельным поддеревом от живого
    /// заезда, чтобы перестройка галереи (removeCar) не сносила машину демо.
    Entity galleryRoot;

    /// Геометрия и материалы, переиспользуемые между поколениями:
    /// создаются один раз, чтобы перестройка галереи не накапливала
    /// меши и материалы (dlib-память вне GC, иначе — утечка и крах).
    Mesh meshBeam = null;
    Mesh meshWheel = null;
    Mesh meshCockpit = null;
    Texture texBeam;
    Material matBeam;
    Material matWheel;
    Material matDriveWheel;
    Material matCockpit;

    /// Держит меш кабины живым (dlib-память вне GC): из него берётся
    /// `meshCockpit`, а asset владеет вершинами.
    private ObjModel cockpitModel_;

    Grammar grammar;
    Random rnd;

    /// Текущая популяция отбора и номер поколения.
    Individual[] population;
    size_t generation;

    EvolutionConfig evolutionConfig;

    enum size_t galleryTop = 5;
    enum float gallerySpacing = 4.0f;
    /// Подальше назад по курсу (backward — константа каркаса): галерея живёт
    /// за линией старта и не сливается с симулируемой машиной.
    enum float galleryBack = 5.0f;

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

    /// V: показывать ли живой заезд текущего поколения (визуализация).
    private bool visualize_;

    /// G-стоп: серию поколений доиграть и остановиться.
    private bool stopRequested_;

    /// Секунд реального времени после схода живого заезда: машина моргает
    /// (видима/скрыта) на этой частоте в ожидании переключения по N.
    private double liveFailTime;
    private enum float liveBlinkHz = 2.0f;

    /// Визуализатор процедурной поверхности (общий shared-кэш с фитнесом).
    private TerrainVisualizer terrainVis;

    /// ОТЛАДКА: виртуальная багги вместо физической — фокус окна со смещением
    /// едет вперёд-влево. Переключение клавишей F отключено; чтобы включить,
    /// верни ветку обработки в update(). Проверка: следует ли окно тайлов
    /// и камера за зоной интереса без всей физики.
    private bool fakeCarMode_;
    private vec3 fakeFocus_ = origin;
    private float fakeCarSpeed_ = 8.0f;

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
        // Гибридная оценка: статический гейт + 2 минуты физического заезда.
        evolutionConfig.simulateSeconds = 120.0;
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

        galleryRoot = addEntity();
        galleryRoot.rotation = rotationQuaternion(Vector3f(1, 0, 0), degtorad(-90.0f));

        meshBeam = New!ShapeCylinder(1.0f, 1.0f, 8, assetManager);
        meshWheel = New!ShapeTorus(0.2f, 0.1f, 16, 8, assetManager);

        matBeam = addMaterial();
        matBeam.baseColorFactor = Color4f(0.55f, 0.55f, 0.62f, 1.0f);
        matBeam.metallicFactor = 0.7f;
        matBeam.baseColorTexture = buildBeamGradientTexture();

        matWheel = addMaterial();
        matWheel.baseColorFactor = Color4f(1, 1, 1, 1);
        matWheel.baseColorTexture = buildTireTexture(Color4f(0.08f, 0.08f, 0.09f, 1.0f));
        matWheel.roughnessFactor = 0.9f;
        matWheel.metallicFactor = 0.0f;

        matDriveWheel = addMaterial();
        matDriveWheel.baseColorFactor = Color4f(1, 1, 1, 1);
        matDriveWheel.baseColorTexture = buildTireTexture(Color4f(0.55f, 0.08f, 0.08f, 1.0f));
        matDriveWheel.roughnessFactor = 0.9f;
        matDriveWheel.metallicFactor = 0.0f;

        // Кабина-корпус: яркий не-металл, чтобы её ориентация читалась визуально
        // на фоне серых балок. Меш один на все сущности (галерея + live).
        cockpitModel_ = loadCockpit();
        meshCockpit = cockpitModel_.mesh;
        meshCockpit.prepareVAO();
        matCockpit = addMaterial();
        matCockpit.baseColorFactor = Color4f(0.95f, 0.6f, 0.1f, 1.0f);
        matCockpit.roughnessFactor = 0.3f;
        matCockpit.metallicFactor = 0.1f;

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

    /// Текстура покрышки: базовая резина `base` + одна светлая радиальная
    /// полоса на всю ширину протектора. Полоса асимметрична по окружности,
    /// поэтому по её повороту видно вращение колеса. UV тора: u — кругом
    /// (окружность), v — поперёк трубы (ширина покрышки).
    private Texture buildTireTexture(const Color4f base)
    {
        const int imgW = 64;
        const int imgH = 8;
        const Color4f stripe = Color4f(0.9f, 0.9f, 0.8f, 1.0f);

        SuperImage img = unmanagedImage(imgW, imgH, 4, 8);
        foreach (y; 0 .. imgH)
            foreach (x; 0 .. imgW)
                img[x, y] = base;

        // Полоса — один сектор окружности (~1/8), во всю ширину покрышки.
        foreach (y; 0 .. imgH)
            foreach (x; imgW / 2 - imgW / 16 .. imgW / 2 + imgW / 16)
                img[x, y] = stripe;

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

        // Камера-орбита плавно ведёт центр массы живой машины; угол обзора
        // остаётся за мышью (повороты/зум не сбрасываются). Точка орбиты в
        // FreeviewComponent инвертирована (см. targetEntity: target = -pos),
        // поэтому передаём позицию с минусом.
        if (freeview !is null)
        {
            if (fakeCarMode_)
            {
                // Виртуальная багги: фокус живёт в базuce каркаса (X = forward,
                // Y = right); в мир dagon/Newton — (fakeFocus_.x, 0, -fakeFocus_.y).
                freeview.setTargetSmooth(-Vector3f(fakeFocus_.x, 0.0f,
                    -fakeFocus_.y));
            }
            else if (livePhysics !is null)
                freeview.setTargetSmooth(-Vector3f(livePhysics.worldFocus));
        }

        // R — всегда вручную: новое 0-е поколение. Вне фоновой эволюции.
        if (jobThread is null && eventManager.keyDown[KEY_R])
        {
            resetPopulation();
            if (visualize_)
            {
                // Живой просмотр продолжается, но по новому 0-му поколению.
                stopLiveCar();
                batch = evaluateStatic(grammar,
                    population.map!(e => e.genotype).array, evolutionConfig);
            }
            buildGallery();
            logGeneration();
            return;
        }

        // G — запуск/остановка эволюции: только сам отбор, без переключения
        // экрана. V — визуализация текущего поколения (живой заезд).
        if (eventManager.keyDown[KEY_G])
            toggleEvolution();
        if (eventManager.keyDown[KEY_V])
            toggleVisualization();

        if (jobThread !is null)
        {
            if (atomicLoad(jobDone))
            {
                jobThread = null;
                cur = batch.res;
                generation++;
                population = cur;
                if (stopRequested_ || gensDone + 1 >= runGens)
                {
                    stopRequested_ = false;
                    if (visualize_)
                        startLiveCar();   // следующий спавн возьмёт последний batch
                }
                else
                {
                    gensDone++;
                    startNextGen();
                    if (visualize_)
                        startLiveCar();   // следующий спавн возьмёт новый batch
                }
                // «5 лучших» перестраиваются на каждом поколении — и в демо,
                // и без него: витрина живёт отдельным поддеревом от live-заезда.
                buildGallery();
                logGeneration();
            }
            else if (visualize_)
                stepLiveCar(t.delta);
        }
        else if (visualize_)
            stepLiveCar(t.delta);

        updateLiveN();

        if (terrainVis !is null)
        {
            if (fakeCarMode_)
            {
                // Диагональ вперёд-влево в базисе каркаса: forward + right·(-1)
                // (сдвиг чисто по плоскости, высота не меняется).
                // Диагональ вперёд-влево в базисе каркаса (forward·(-1) по Y,
                // right·(-1) по X): сдвиг чисто по плоскости, высота плоская.
                fakeFocus_ = fakeFocus_
                    + vec3(0.0f, -fakeCarSpeed_ * t.delta, 0.0f)
                    - vec3(0.5f * fakeCarSpeed_ * t.delta, 0.0f, 0.0f);
                terrainVis.updateFocus(fakeFocus_);
            }
            else
                terrainVis.update(livePhysics);
        }
    }

    /// G: запустить отбор поколений либо остановить после текущего поколения.
    private void toggleEvolution()
    {
        if (jobThread !is null)
        {
            stopRequested_ = true;
            return;
        }

        // Сброс до старта: иначе устаревший true от прошлого задания
        // мгновенно применяет результат и гасит live-заезд.
        atomicStore(jobDone, false);
        runCfg = evolutionConfig;
        cur = population;
        // G — toggle: эволюция идёт, пока её снова не остановит G.
        runGens = size_t.max;
        gensDone = 0;
        stopRequested_ = false;
        startNextGen();
        logGeneration();
    }

    /// V: показать/спрятать живой заезд текущего поколения.
    private void toggleVisualization()
    {
        visualize_ = !visualize_;
        if (visualize_)
        {
            // Свежий batch по текущей популяции (статически, без физического
            // заезда): и до первого G, и после R batch может не совпадать с
            // population, а живому просмотру нужны его needPhysics.
            if (jobThread is null)
                batch = evaluateStatic(grammar,
                    population.map!(e => e.genotype).array, evolutionConfig);
            startLiveCar();
        }
        else
        {
            stopLiveCar();
            buildGallery();
        }
    }

    /// Переключение симулируемой особи — только вручную, по N.
    /// keyPressed — защёлка события; игнорируем повторы, пока клавиша
    /// не отпущена (одно переключение на одно нажатие).
    private void updateLiveN()
    {
        const bool nHeld = eventManager.keyPressed[KEY_N];
        if (nHeld && !nWasHandled_ && visualize_ && livePhysics !is null)
        {
            nWasHandled_ = true;
            if (showNextLiveBuggy())
                updateLiveCar();
        }
        if (!nHeld)
            nWasHandled_ = false;
    }

    /// Строит партию поколения на главном потоке и запускает её физику в фоне.
    private void startNextGen()
    {
        auto children = buildNextGeneration(grammar, cur, runCfg, rnd);
        batch = evaluateStatic(grammar, children, runCfg);
        jobGen = generation + 1;
        // Новый batch — следующее поколение: сброс live-цикла. Текущая машина
        // продолжает ехать (V не рвётся), а следующий спавн (после схода или
        // по N) берёт машины уже нового поколения.
        liveBatch = null;
        liveBatchIdx = 0;
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
            auto physics = new BuggyPhysics(new Buggy(placedFrame(frame)),
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
            liveFailTime = 0.0;

            // Мгновенно садим камеру на нового багги: при старте заезда и при
            // каждом N smoothTarget догонял бы цель несколько секунд с прежней
            // позиции — за это время машина уезжает по экрану в одну сторону,
            // а террайн (окно уже перецентрировано) выглядит «едущим » в другую.
            if (freeview !is null)
                freeview.setTarget(-Vector3f(livePhysics.worldFocus));

            // По одному цилиндру на каждую физическую балку каркаса: порядок
            // совпадает с BeamState[] из beamStates() (по Frame.beams).
            foreach (b; frame.beams)
            {
                const beam = cast(Beam) b;
                if (beam is null)
                    continue;
                const float len =
                    (frame.nodes[b.b].pos - frame.nodes[b.a].pos).length;
                auto e = addEntity(carRoot);
                e.drawable = meshBeam;
                e.material = matBeam;
                e.scaling = Vector3f(beam.radius, len, beam.radius);
                liveCar ~= e;
            }

            foreach (a; frame.anchors)
            {
                auto e = addEntity(carRoot);
                e.drawable = meshWheel;
                e.material = a.kind == AnchorKind.motorWheel ? matDriveWheel : matWheel;
                // Масштаб под генетический радиус колеса, как и в витрине.
                const float s = a.radius / wheelRadius;
                e.scaling = Vector3f(s, s, s);
                liveCar ~= e;
            }

            // Кабина: живёт в конце liveCar после всех балок и колёс; позицию
            // и ориентацию на каждом шаге даёт мастер-тело (cockpitState).
            auto eCab = addEntity(carRoot);
            eCab.drawable = meshCockpit;
            eCab.material = matCockpit;
            liveCar ~= eCab;
            return true;
        }

        liveBatchIdx = liveBatch.length;
        goto reload;
    }

    private void stepLiveCar(const double dt)
    {
        if (livePhysics is null)
        {
            if (showNextLiveBuggy())
                updateLiveCar();
            return;
        }

        if (liveFailed_)
        {
            // Сход: физика заморожена (машина стоит на месте), остаток заезда
            // машина моргает 2 Гц до ручного переключения по N.
            liveFailTime += dt;
            setLiveVisible((cast(int)(liveFailTime * 2.0 * liveBlinkHz) & 1) == 0);
            return;
        }

        livePhysics.step(physicsDt, 1.0f);
        liveSimTime += physicsDt;

        const stepFailure = runFailure(livePhysics);
        if (stepFailure.length)
        {
            liveFailed_ = true;
            liveFailTime = 0.0;
            writefln("live: заезд оборван (%s) — машина заморожена, ждём N", stepFailure);
        }
        else if (livePhysics.cabinTouchesGround())
        {
            liveFailed_ = true;
            liveFailTime = 0.0;
            writefln("live: кабина коснулась земли — машина заморожена, ждём N");
        }

        updateLiveCar();
    }

    private void setLiveVisible(const bool on)
    {
        foreach (e; liveCar)
            e.visible = on;
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

        // Кабина — последняя сущность liveCar; состояние от мастер-тела.
        const size_t cabIdx = beams.length + wheels.length;
        if (cabIdx < liveCar.length)
        {
            const BodyState c = livePhysics.cockpitState;
            liveCar[cabIdx].position = c.position;
            liveCar[cabIdx].rotation = c.orientation;
        }

        carRoot.updateTransformationTopDown();
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
        foreach (e; galleryRoot.children)
            toRemove ~= e;

        // Снимаем детей витрины с корня и из мира: иначе сущности навечно
        // остаются в galleryRoot.children и с каждым поколением галерея
        // накапливает сотни сущностей в сцене.
        foreach (e; toRemove)
        {
            removeEntity(e);
            galleryRoot.removeChild(e);
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
            // Buggy раскладывает каркас сам (центр в нуле, колёса на земле);
            // витрине остаётся только сдвиг в свою полосу по X (display-only).
            auto buggy = new Buggy(placedFrame(f.get.frame));
            drawBuggy(buggy, vec3(laneX, 0.0f, 0.0f) + backward * galleryBack);
        }

        galleryRoot.updateTransformationTopDown();
    }

    /// Рисует машину из Buggy: статичные балки и колёса. `off` — сдвиг витрины
    /// (полоса по X); гравитационный «на старт» каркас уже несёт в себе.
    private void drawBuggy(const Buggy buggy, const vec3 off)
    {
        foreach (b; buggy.frame.beams)
        {
            const beam = cast(Beam) b;
            if (beam is null) // эфемерные балки не рисуются
                continue;
            const a = buggy.frame.nodes[b.a].pos + off;
            const b2 = buggy.frame.nodes[b.b].pos + off;
            const dir = b2 - a;
            const float length = dir.length;
            if (length < 1e-5f)
                continue;

            auto e = addEntity(galleryRoot);
            e.drawable = meshBeam;
            e.material = matBeam;
            e.position = (a + b2) * 0.5f;
            e.rotation = rotationBetween(Vector3f(0, 1, 0), dir / length);
            e.scaling = Vector3f(beam.radius, length, beam.radius);
        }

        foreach (anchor; buggy.frame.anchors)
        {
            const nodeCar = buggy.frame.nodes[anchor.node].pos;
            const pos = nodeCar + off;
            const vec3 axle = wheelAxle(frameForward, frameUp, nodeCar);
            final switch (anchor.kind)
            {
                case AnchorKind.wheel:
                    addWheel(pos, axle, matWheel, anchor.radius);
                    break;
                case AnchorKind.motorWheel:
                    addWheel(pos, axle, matDriveWheel, anchor.radius);
                    break;
            }
        }

        // Кабина: меш уже в координатах каркаса, центр меша (0,0,0 OBJ) — ЦМ,
// совмещён с узлом 0. Поворот тождественный — прежняя компенсация toCarRot
// больше не нужна; сдвига на −seed, как в старой схеме, нет.
        auto eCab = addEntity(galleryRoot);
        eCab.drawable = meshCockpit;
        eCab.material = matCockpit;
        eCab.position = buggy.frame.nodes[0].pos + off;
        eCab.rotation = Quaternionf.identity;
    }

    private void addWheel(const vec3 pos, const vec3 axle, Material mat,
        const float radius)
    {
        auto e = addEntity(galleryRoot);
        e.drawable = meshWheel;
        e.material = mat;
        e.position = pos;
        e.rotation = rotationBetween(Vector3f(0, 1, 0), axle);
        // Масштаб по генетическому радиусу: тор рисуется под базовый
        // `wheelRadius`, обод вытягивается на свой размер.
        const float s = radius / wheelRadius;
        e.scaling = Vector3f(s, s, s);
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