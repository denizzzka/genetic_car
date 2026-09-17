module viewer.viewer;

import dagon;
import dagon.core.keycodes;
import dagon.core.time;
import std.algorithm : min, sort;
import std.random;
import std.stdio : writefln;
import frame.frame;
import genetics;

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

    /// Объём популяции, размер витрины и поколений за нажатие G.
    enum size_t populationSize = 20;
    enum size_t galleryTop = 5;
    enum size_t generationsPerPress = 10;
    enum float gallerySpacing = 3.0f;

    override void afterLoad()
    {
        eventManager.trackUpDownState = true;
        grammar = buggyGrammar();
        rnd = Random(42);

        auto camera = addCamera();
        auto freeview = New!FreeviewComponent(eventManager, camera);
        freeview.setZoom(7.0f);
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
        population = evaluatePopulation(grammar, seedPopulation(grammar, populationSize));
        generation = 0;
    }

    override void update(Time t)
    {
        super.update(t);

        if (eventManager.keyDown[KEY_R])
        {
            resetPopulation();
            buildGallery();
            logGeneration();
        }
        else if (eventManager.keyDown[KEY_G])
        {
            population = evolve(grammar, population, generationsPerPress, rnd);
            generation += generationsPerPress;
            buildGallery();
            logGeneration();
        }
        else if (eventManager.keyDown[KEY_M])
        {
            population = evolve(grammar, population, 1, rnd);
            generation += 1;
            buildGallery();
            logGeneration();
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
            drawBuggy(f.get, laneX);
        }
    }

    /// Рисует каркас как статичные балки и колёса в своей полосе laneX.
    private void drawBuggy(const Frame frame, float laneX)
    {
        const off = laneOffset(frame, laneX);

        foreach (b; frame.beams)
        {
            const a = frame.nodes[b.a].pos + off;
            const b2 = frame.nodes[b.b].pos + off;
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

        foreach (anchor; frame.anchors)
        {
            const pos = frame.nodes[anchor.node].pos + off;
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
            lift = 0.3f - minZ; // 0.3 — радиус колеса (physics_world.wheelRadius)
            if (lift < 0.0f)
                lift = 0.0f;
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