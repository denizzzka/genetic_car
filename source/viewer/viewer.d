module viewer.viewer;

import dagon;
import dagon.core.keycodes;
import dagon.core.time;
import core.thread : Thread;
import core.atomic : atomicStore, atomicLoad;
import std.algorithm : min, map, reduce, sort;
import std.math : atan2;
import std.range : evenChunks;
import std.array : array;
import std.random;
import std.datetime.stopwatch: StopWatch, AutoStart;
import std.stdio : writefln;
import frame.frame;
import frame.frame : frameForward = forward, frameUp = up;
import frame.cockpit : loadCockpit;
import frame.objmesh : ObjModel;
import genetics;
import physics_world;
import physics_world.engine : PhysWorld;
import viewer.scene : carToScenePos;
import viewer.startaxes : buildStartAxes;
import viewer.terrainvisualizer;
import viewer.meshes;

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
    Mesh meshTrack;
    Texture texBeam;
    Material matBeam;
    Material matWheel;
    Material matDriveWheel;
    Material matCockpit;
    Material matEphemeral;
    Material matTrack;

    /// Держит меш кабины живым (dlib-память вне GC): из него берётся
    /// `meshCockpit`, а asset владеет вершинами.
    private ObjModel cockpitModel_;

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
    private PhysWorld liveWorld;
    private Entity[] liveCar;
    private double liveSimTime;
    private double liveRunSeconds;

    /// D: дебажное отображение — эфемерные балки красными цилиндрами,
    /// кабина скрыта. Повторное нажатие возвращает обычный вид.
    private bool debugSkeleton_;

    /// Красные цилиндры эфемерных балок живого заезда (двигаются с мастером).
    private Entity[] liveEphemeral;

    /// Концы эфемерных балок в координатах каркаса: мировые позиции
    /// каждый шаг даёт мастер (framePointWorld).
    private vec3[2][] liveEph; // концы

    /// Индекс кабины в liveCar — для показа/скрытия при переключении D.
    private size_t liveCabIdx_ = size_t.max;

    /// Тонкий радиус дебажных цилиндров эфемерных балок, м.
    enum float ephVisRadius = 0.01f;

    /// Следы колёс живого заезда: пятно на земле, когда колесо на ней стоит и
    /// прошло с прошлого пятна trackStep. Пул по кругу — за длинный заезд
    /// пятна больше, чем trackCap, и самые старые просто сменяются новыми.
    private Entity[] tracks;
    private vec3[] trackLast;
    private bool[] trackSeen;
    private size_t trackNext;

    /// След вдоль курса и поперёк, м; шаг между пятнами, м; ёмкость пула.
    /// Поперёк пятно равно ширине покрышки, иначе след уже самой покрышки.
    enum float trackMarkLen = 0.1f;
    enum float trackMarkWidth = wheelWidth;
    enum float trackStep = 0.25f;
    enum size_t trackCap = 512;

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
    /// Кандидаты текущей партии по убыванию фитнеса: перебор в порядке
    /// needPhysics доходит до пригодной машины через десятки усадок.
    private size_t[] liveOrder;

    // Гонка поколений: строится на главном потоке, физика считается в фоне.
    private Individual[] cur;
    private size_t runGens;
    private size_t gensDone;
    private size_t jobGen;
    private PhysicsBatch batch;
    private EvolutionConfig runCfg;
    /// `batch` собран из текущей `population` — пересчитывать его для живого
    /// просмотра незачем, а пересчёт ставит эволюцию на десятки секунд.
    private bool batchIsPopulation_;

    /// Фазы последнего поколения, с: рост (buildNextGeneration), отбор
    /// (evaluateStatic) и фоновая физика — печатаются одной строкой.
    private GenerationTiming jobTiming_;

    override void afterLoad()
    {
        // BuggyScene создан через New! (dlib): GC не сканирует dlib-память.
        // Регистрируем объект сцены как GC-диапазон, чтобы ссылки в его полях
        // (population, terrainVis) не уносились сборщиком.
        // afterLoad, как и update, может быть вызван позже при загрузке сцены.
        import core.memory: GC;
        GC.addRange(cast(void*)this, __traits(classInstanceSize, BuggyScene));

        eventManager.trackUpDownState = true;
        rnd = Random(42);
        // Гибридная оценка: статический гейт + 2 минуты физического заезда.
        evolutionConfig.simulateSeconds = 120.0;
        liveRunSeconds = evolutionConfig.simulateSeconds;
        // Виден ли ход заездов в stdout (по особам и поколениям).
        evolutionConfig.logPhysics = true;

        // Мир показа берём из пула, поэтому фоновым остаётся на один меньше:
        // пик миров держится на потолке, а вьюер не ждёт освобождения за ними.
        setWorkPoolSize(pooledWorldCount() - 1);

        writefln("engine: %s", selectedEngineName);

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
        // Покрышка — та же полая труба, что и в физике: внешний радиус, ширина
        // и число граней берутся одни и те же.
        meshWheel = buildWheelMesh(assetManager);

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

        // Эфемерные балки в дебаге: тонкий красный металл, чтобы скелет
        // читался на фоне серых масс. Текстура-градиент не нужна.
        matEphemeral = addMaterial();
        matEphemeral.baseColorFactor = Color4f(1.0f, 0.12f, 0.12f, 1.0f);
        matEphemeral.roughnessFactor = 0.7f;
        matEphemeral.metallicFactor = 0.1f;

        // След колёса: тёмное матовое пятно на земле, один квад на все следы.
        meshTrack = buildTrackMarkMesh(assetManager);
        matTrack = addMaterial();
        matTrack.baseColorFactor = Color4f(0.12f, 0.11f, 0.10f, 1.0f);
        matTrack.roughnessFactor = 1.0f;
        matTrack.metallicFactor = 0.0f;
        buildTracks();

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

        buildStartAxes(this, sharedTerrain());

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
        population = evaluatePopulation(seedPopulation(EvolutionConfig.populationSize));
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
                freeview.setTargetSmooth(-carToScenePos(fakeFocus_));
            }
            else if (livePhysics !is null)
                freeview.setTargetSmooth(-carToScenePos(livePhysics.worldFocus));
        }

        // R — всегда вручную: новое 0-е поколение. Вне фоновой эволюции.
        if (jobThread is null && eventManager.keyDown[KEY_R])
        {
            resetPopulation();
            if (visualize_)
            {
                // Живой просмотр продолжается, но по новому 0-му поколению.
                stopLiveCar();
                // Каркасы текущей популяции уже выращены — растить их заново
                // только ради пересчёта фитнеса незачем.
                batch = evaluateStatic(
                    population.map!(e => Organism(e.chromosome, e.frame)).array,
                    evolutionConfig);
                batchIsPopulation_ = true;
            }
            else
                batchIsPopulation_ = false;
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
        if (eventManager.keyDown[KEY_D])
            toggleDebugSkeleton();

        if (jobThread !is null)
        {
            if (atomicLoad(jobDone))
            {
                jobThread = null;
                cur = batch.res;
                generation++;
                population = cur;
                batchIsPopulation_ = true;
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
                // Диагональ вперёд-влево в базисе каркаса: forward·(-1) по Y,
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
            if (jobThread is null && !batchIsPopulation_)
            {
                // Каркасы текущей популяции уже выращены — растить их заново
                // только ради пересчёта фитнеса незачем.
                batch = evaluateStatic(
                    population.map!(e => Organism(e.chromosome, e.frame)).array,
                    evolutionConfig);
                batchIsPopulation_ = true;
            }
            startLiveCar();
        }
        else
        {
            stopLiveCar();
            buildGallery();
        }
    }

    /// D: дебажный слой эфемерных балок вместо кабины и обратно.
    private void toggleDebugSkeleton()
    {
        debugSkeleton_ = !debugSkeleton_;
        rebuildLiveDebug();
        buildGallery();
    }

    /// Перестройка дебажного слоя живого заезда: эфемерные балки заново,
    /// кабина — в зависимости от режима.
    private void rebuildLiveDebug()
    {
        removeLiveEphemeral();
        if (debugSkeleton_ && livePhysics !is null)
            buildLiveEphemeral();
        setLiveCabinVisible(!debugSkeleton_);
    }

    /// Показ/скрытие кабины живого заезда — кабина последняя в liveCar.
    private void setLiveCabinVisible(const bool on)
    {
        if (liveCabIdx_ < liveCar.length)
            liveCar[liveCabIdx_].visible = on;
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
        StopWatch swGen = StopWatch(AutoStart.yes);
        auto children = buildNextGeneration(cur, runCfg, rnd);
        const double growSec = swGen.peek.total!"seconds";
        batch = evaluateStatic(children, runCfg);
        batchIsPopulation_ = false;   // это следующее поколение, не текущее
        jobTiming_.grow = growSec;
        jobTiming_.eval = swGen.peek.total!"seconds" - growSec;
        jobGen = generation + 1;
        // Новый batch — следующее поколение: сброс live-цикла. Текущая машина
        // продолжает ехать (V не рвётся), а следующий спавн (после схода или
        // по N) берёт машины уже нового поколения.
        liveBatch = null;
        liveBatchIdx = 0;
        liveOrder = null;
        atomicStore(jobDone, false);
        jobThread = new Thread({
            StopWatch swPhys = StopWatch(AutoStart.yes);
            runPhysics(batch, runCfg, jobGen);
            swPhys.stop();
            jobTiming_.physics = swPhys.peek.total!"seconds";
            atomicStore(jobDone, true);
        });
        jobThread.isDaemon = true;
        jobThread.start();
    }

    private void startLiveCar()
    {
        liveBatch = null;
        liveBatchIdx = 0;
        liveOrder = null;
    }

    private bool showNextLiveBuggy()
    {
        // Ищем пригодную машину в текущей партии; если партия кончилась —
        // берём свежей из batch.needPhysics и начинаем заново.
        while (true)
        {
            if (liveBatch is null || liveBatchIdx >= liveOrder.length)
            {
                auto next = batch.needPhysics;
                if (next is liveBatch || next.length == 0)
                    return false;
                liveBatch = next;
                liveBatchIdx = 0;
                liveOrder = new size_t[next.length];
                foreach (k; 0 .. next.length)
                    liveOrder[k] = k;
                liveOrder.sort!((a, b) => batch.res[batch.physIdx[b]].fitness
                    > batch.res[batch.physIdx[a]].fitness);
            }

            foreach (attempt; liveOrder[liveBatchIdx .. $])
            {
                Frame frame = liveBatch[attempt].frame;
                if (frame.anchors.length < 2 || !canDrive(frame))
                    continue;
                spawnLiveBuggy(frame);
                liveBatchIdx = attempt + 1;
                return true;
            }

            liveBatchIdx = liveOrder.length;
        }
    }

    /// Ставит машину на старт и запускает живой заезд. Падение с `dropHeight`
    /// входит в заезд, поэтому заведомо неустойчивая машина видна целиком:
    /// сколько она простояла, столько и ехала.
    private void spawnLiveBuggy(const Frame frame)
    {
        // Старый мир возвращаем в пул до запроса нового: иначе вьюер на
        // время держит два мира, а пул конечен.
        disposeLivePhysics();
        removeLiveEntities();
        clearTracks();

        import physics_world.engineselect: simLock;
        if (auto lk = simLock)
            synchronized (lk)
                return spawnLiveBuggyRun(frame);
        spawnLiveBuggyRun(frame);
    }

    private void spawnLiveBuggyRun(const Frame frame)
    {
        liveWorld = acquireWorld();
        livePhysics = new BuggyPhysics(new Buggy(placedFrame(frame)),
            liveWorld, sharedTerrain());

        liveSimTime = 0.0;
        liveFailed_ = false;
        liveFailTime = 0.0;

        // Мгновенно садим камеру на нового багги: при старте заезда и при
        // каждом N smoothTarget догонял бы цель несколько секунд с прежней
        // позиции — за это время машина уезжает по экрану в одну сторону,
        // а террайн (окно уже перецентрировано) выглядит «едущим » в другую.
        if (freeview !is null)
            freeview.setTarget(-carToScenePos(livePhysics.worldFocus));

        // По одному цилиндру на каждую физическую балку каркаса: порядок
        // совпадает с BeamState[] из beamStates() (по Frame.beams).
        foreach (b; frame.beams)
        {
            const beam = cast(Beam) b;
            if (beam is null)
                continue;
            const float len = (frame.nodes[b.b].pos - frame.nodes[b.a].pos).length;
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
            // Меш построен под базовый `wheelRadius`, генетический радиус
            // отличается равномерным масштабом: пропорции трубы те же.
            const float s = a.radius / wheelRadius;
            e.scaling = Vector3f(s, s, s);
            liveCar ~= e;
        }

        // Кабина: живёт в конце liveCar после всех балок и колёс; позицию
        // и ориентацию на каждом шаге даёт мастер-тело (cockpitState).
        auto eCab = addEntity(carRoot);
        eCab.drawable = meshCockpit;
        eCab.material = matCockpit;
        liveCabIdx_ = liveCar.length;
        liveCar ~= eCab;
        if (debugSkeleton_)
        {
            eCab.visible = false;
            buildLiveEphemeral();
        }
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
        if (stepFailure != RunOutcome.none)
        {
            liveFailed_ = true;
            liveFailTime = 0.0;
            writefln("live: заезд оборван (%s) — машина заморожена, ждём N",
                cast(string) stepFailure);
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
        removeLiveEphemeral();
        foreach (e; liveCar)
        {
            removeEntity(e);
            carRoot.removeChild(e);
        }
        liveCar.length = 0;
        liveCabIdx_ = size_t.max;
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

        updateLiveEphemeral();

        dropTracks(wheels);

        carRoot.updateTransformationTopDown();
    }

    /// Пул пятен следов: сущности создаются один раз и дальше только
    /// переставляются, чтобы заезд не плодил геометрию в кадре.
    private void buildTracks()
    {
        tracks = new Entity[trackCap];
        foreach (i; 0 .. trackCap)
        {
            auto e = addEntity(carRoot);
            e.drawable = meshTrack;
            e.material = matTrack;
            e.scaling = Vector3f(trackMarkLen, trackMarkWidth, 1.0f);
            e.castShadow = false;
            e.visible = false;
            tracks[i] = e;
        }
    }

    /// Следы прошлой машины — в пул: пятна переиспользуются по кругу.
    private void clearTracks()
    {
        foreach (e; tracks)
            e.visible = false;
        foreach (ref seen; trackSeen)
            seen = false;
        trackNext = 0;
    }

    /// Пятно следа на колесо, которое стоит на земле и отошло от прошлого
    /// пятна на полметра. Курс пятна берём с машины: без поворота след на
    /// виражах читается как случайный пунктир.
    private void dropTracks(const BodyState[] wheels)
    {
        if (wheels.length > trackLast.length)
        {
            trackLast.length = wheels.length;
            trackSeen.length = wheels.length;
        }

        const Frame fr = livePhysics.frame;
        auto surface = sharedTerrain();
        const Vector3f fwd = livePhysics.cockpitState.orientation
            .rotate(frameForward);
        const float yaw = atan2(fwd.y, fwd.x);
        foreach (i, s; wheels)
        {
            const float r = i < fr.anchors.length ? fr.anchors[i].radius
                : wheelRadius;
            const float h = surface.heightAt(s.position);
            if (heightOf(s.position) - r > h + 0.05f)
                continue;
            if (trackSeen[i] && (s.position - trackLast[i]).length < trackStep)
                continue;

            auto e = tracks[trackNext];
            trackNext = (trackNext + 1) % trackCap;
            e.visible = true;
            e.position = vec3(s.position.x, s.position.y, h + 0.02f);
            e.rotation = rotationQuaternion(Vector3f(0, 0, 1), yaw);
            trackLast[i] = s.position;
            trackSeen[i] = true;
        }
    }

    /// Эфемерные балки каркаса красным слоем: концы едут на мастере, поэтому
    /// запоминаем только координаты каркаса и считаем мировые на каждом шаге.
    private void buildLiveEphemeral()
    {
        if (livePhysics is null)
            return;
        const Frame fr = livePhysics.frame;
        foreach (b; fr.beams)
        {
            if (cast(const Beam) b !is null)
                continue;
            const a = fr.nodes[b.a].pos;
            const c = fr.nodes[b.b].pos;
            if ((c - a).length < 1e-5f)
                continue;
            auto e = addEntity(carRoot);
            e.drawable = meshBeam;
            e.material = matEphemeral;
            liveEphemeral ~= e;
            liveEph ~= [a, c];
        }
    }

    /// Мировые позиции эфемерных балок от мастера — как в updateLiveCar.
    private void updateLiveEphemeral()
    {
        if (livePhysics is null)
            return;
        foreach (i, ends; liveEph)
        {
            if (i >= liveEphemeral.length)
                break;
            const a = livePhysics.framePointWorld(ends[0]);
            const c = livePhysics.framePointWorld(ends[1]);
            placeBeam(liveEphemeral[i], a, c, ephVisRadius);
        }
    }

    private void removeLiveEphemeral()
    {
        foreach (e; liveEphemeral)
        {
            removeEntity(e);
            carRoot.removeChild(e);
        }
        liveEphemeral.length = 0;
        liveEph.length = 0;
    }

    /// Снос живого заезда и возврат его мира в пул.
    private void disposeLivePhysics()
    {
        // Разбор сцены идёт под тем же замком, что и заезды воркеров: Jolt
        // не переносит, когда один мир разбирают, пока другой шагает.
        import physics_world.engineselect: simLock;
        if (auto lk = simLock)
            synchronized (lk)
                return disposeLivePhysicsRun();
        disposeLivePhysicsRun();
    }

    private void disposeLivePhysicsRun()
    {
        if (livePhysics !is null)
        {
            livePhysics.dispose();
            livePhysics = null;
        }
        if (liveWorld !is null)
        {
            releaseWorld(liveWorld);
            liveWorld = null;
        }
    }

    private void stopLiveCar()
    {
        removeLiveEntities();
        disposeLivePhysics();
        liveBatch = null;
        liveBatchIdx = 0;
    }

    private void logGeneration()
    {
        writefln("gen %d: best=%.4f mean=%.4f pop=%d %s",
            generation, bestFitness(population), meanFitness(population),
            population.length, jobTiming_.toPhases());
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

    private void buildGallery()
    {
        removeCar();

        const size_t strata = min(cast(size_t) galleryTop, population.length);
        Individual[] picks;
        foreach (members; population[0 .. $].evenChunks(strata))
            picks ~= members.reduce!((a, b) => a.fitness > b.fitness ? a : b);

        const float firstX = (picks.length - 1) * 0.5f * gallerySpacing;

        foreach (i, pick; picks)
        {
            auto f = develop(pick.chromosome);
            if (f.isNull)
                continue;
            const float laneX = i * gallerySpacing - firstX;
            // Buggy раскладывает каркас сам (центр в нуле, колёса на земле);
            // витрине остаётся только сдвиг в свою полосу по X (display-only).
            auto buggy = new Buggy(placedFrame(f.get));
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
            if (beam is null)
            {
                if (debugSkeleton_)
                    drawEphemeralBeam(buggy.frame, b.a, b.b, off);
                continue;
            }
            const a = buggy.frame.nodes[b.a].pos + off;
            const b2 = buggy.frame.nodes[b.b].pos + off;
            if ((b2 - a).length < 1e-5f)
                continue;

            auto e = addEntity(galleryRoot);
            e.drawable = meshBeam;
            e.material = matBeam;
            placeBeam(e, a, b2, beam.radius);
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
        // совмещён с узлом 0; сдвига на −seed, как в старой схеме, нет.
        auto eCab = addEntity(galleryRoot);
        eCab.drawable = meshCockpit;
        eCab.material = matCockpit;
        eCab.position = buggy.frame.nodes[0].pos + off;
        eCab.rotation = Quaternionf.identity;
        if (debugSkeleton_)
            eCab.visible = false;
    }

    /// Тонкий красный цилиндр эфемерной балки витрины (зеркало live-слоя).
    private void drawEphemeralBeam(const Frame fr, const size_t na, const size_t nb,
        const vec3 off)
    {
        const a = fr.nodes[na].pos + off;
        const c = fr.nodes[nb].pos + off;
        if ((c - a).length < 1e-5f)
            return;

        auto e = addEntity(galleryRoot);
        e.drawable = meshBeam;
        e.material = matEphemeral;
        placeBeam(e, a, c, ephVisRadius);
    }

    /// Ставит цилиндр вдоль отрезка a→c: середина — позиция, ось цилиндра
    /// (локальный +Y) совпадает с направлением, длина — масштаб по Y.
    private void placeBeam(Entity e, const vec3 a, const vec3 c, const float radius)
    {
        const dir = c - a;
        const float length = dir.length;
        if (length < 1e-5f)
            return;
        e.position = (a + c) * 0.5f;
        e.rotation = rotationBetween(Vector3f(0, 1, 0), dir / length);
        e.scaling = Vector3f(radius, length, radius);
    }

    private void addWheel(const vec3 pos, const vec3 axle, Material mat,
        const float radius)
    {
        auto e = addEntity(galleryRoot);
        e.drawable = meshWheel;
        e.material = mat;
        e.position = pos;
        e.rotation = rotationBetween(Vector3f(0, 1, 0), axle);
        // Меш построен под базовый `wheelRadius`, генетический радиус
        // отличается равномерным масштабом: пропорции трубы те же.
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