module viewer.viewer;

import dagon;
import dagon.core.keycodes;
import dagon.core.time;
import std.algorithm : min;
import std.random;
import std.typecons : Nullable;
import car.car;
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

    /// Геометрия и материалы, переиспользуемые между кадрами мутаций:
    /// создаются один раз, чтобы повторное нажатие M не накапливало
    /// меши и материалы (dlib-память вне GC, иначе — утечка на каждый
    /// кадр и крах после десятков нажатий).
    Mesh meshBeam = null;
    Mesh meshWheel = null;
    Material matBeam;
    Material matWheel;
    Material matDriveWheel;

    Grammar grammar;
    Random rnd;
    Buggy current;
    Genotype currentGenome;

    /// Физика машины и сущности, синхронизируемые с телами.
    /// Позиции/ориентации тел копируются в сущности в шаге физики.
    CarPhysics physics;
    Entity[] beamEntities;
    Entity[] wheelEntities;

    /// Аккумулятор фиксированного шага симуляции.
    private double accumulator = 0.0;
    private enum double fixedDt = 1.0 / 60.0;

    override void afterLoad()
    {
        eventManager.trackUpDownState = true;
        grammar = buggyGrammar();
        rnd = Random(42);

        auto camera = addCamera();
        auto freeview = New!FreeviewComponent(eventManager, camera);
        freeview.setZoom(4.0f);
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

        currentGenome = startGenome(grammar);
        auto frame = currentFrame();
        current = new Buggy(frame, groundOffset(frame));
        buildCar(current);

        auto ePlane = addEntity();
        ePlane.drawable = New!ShapePlane(10.0f, 10.0f, 1, assetManager);

        /*
        BuggyScene is dlib-allocated (New!), so the GC can't see
        references to objects (grammar, currentGenome, current) stored
        in its fields. After enough GC pressure, these objects get
        collected, and the next access SIGSEGVs.

        It is need to register the scene's memory as a GC range so
        the GC scans its fields for pointers.
        */
        import core.memory: GC;
        GC.addRange(cast(void*)this, __traits(classInstanceSize, BuggyScene));
    }

    private Frame currentFrame()
    {
        auto may = develop(grammar, currentGenome);
        assert(!may.isNull, "encoded buggy frame must develop");
        return may.get;
    }

    override void update(Time t)
    {
        super.update(t);

        if (eventManager.keyDown[KEY_R])
        {
            currentGenome = startGenome(grammar);
            removeCar();
            auto frame = currentFrame();
            current = new Buggy(frame, groundOffset(frame));
            buildCar(current);
        }
        else if (eventManager.keyDown[KEY_M])
        {
            bool ok;
            Frame f;
            auto candidate = currentGenome;
            foreach (_; 0 .. 100)
            {
                candidate = currentGenome.dup;
                mutate(candidate, 1 + uniform(0u, 3u, rnd), rnd);
                auto may = develop(grammar, candidate);
                if (!may.isNull)
                {
                    f = may.get;
                    ok = true;
                    break;
                }
            }
            if (ok)
            {
                currentGenome = candidate;
                removeCar();
                current = new Buggy(f, groundOffset(f));
                buildCar(current);
            }
        }

        if (physics !is null)
            stepPhysics(t.delta);
    }

    /// Компенсирующее смещение, приводящее каркас к началу координат.
    ///
    /// Горизонтально (X, Y) каркас центрируется по среднему узлов. Вертикально
    /// (Z) каркас поднимается так, чтобы нижняя точка самого низкого колеса
    /// легла на землю (z == 0 в координатах машины). Иначе из-за
    /// центрирования по средней высоте машина наполовину в земле.
    private vec3 groundOffset(const Frame f)
    {
        vec3 c = vec3(0.0f);

        foreach (n; f.nodes)
            c += n.pos;

        if (f.nodes.length > 0)
            c /= f.nodes.length;

        float minZ = float.max;
        foreach (a; f.anchors)
            minZ = min(minZ, f.nodes[a.node].pos.z);

        float lift = wheelRadius - minZ;
        if (lift < 0.0f)
            lift = 0.0f;

        return vec3(-c.x, -c.y, lift);
    }

    private void removeCar()
    {
        if (physics !is null)
        {
            physics.dispose();
            physics = null;
        }

        Entity[] toRemove;
        foreach (e; carRoot.children)
        {
            toRemove ~= e;
        }

        // Снимаем детей с корня и из мира: иначе сущности навечно
        // остаются в carRoot.children и с каждым нажатием M каркас
        // накапливает десятки сущностей в сцене.
        foreach (e; toRemove)
        {
            removeEntity(e);
            carRoot.removeChild(e);
        }
    }

    private void buildCar(const Buggy car)
    {
        const frame = car.frame;
        const off = car.offset;

        physics = new CarPhysics(frame, off);
        beamEntities.length = 0;
        wheelEntities.length = 0;

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
            beamEntities ~= e;
        }

        foreach (anchor; frame.anchors)
        {
            const pos = frame.nodes[anchor.node].pos + off;
            final switch (anchor.kind)
            {
                case AnchorKind.wheel:
                    addWheel(pos);
                    break;
                case AnchorKind.motorWheel:
                    addDriveWheel(pos);
                    break;
            }
        }
    }

    private void addWheel(const vec3 pos)
    {
        auto e = addEntity(carRoot);
        e.drawable = meshWheel;
        e.material = matWheel;
        e.position = pos;
        e.rotation = rotationBetween(Vector3f(0, 1, 0), Vector3f(1, 0, 0));
        wheelEntities ~= e;
    }

    private void addDriveWheel(const vec3 pos)
    {
        auto e = addEntity(carRoot);
        e.drawable = meshWheel;
        e.material = matDriveWheel;
        e.position = pos;
        e.rotation = rotationBetween(Vector3f(0, 1, 0), Vector3f(1, 0, 0));
        wheelEntities ~= e;
    }

    /// Фиксированный шаг физики с накоплением dt, затем синхронизация
    /// трансформов сущностей с телами. Управление: W — газ вперёд, S — назад.
    private void stepPhysics(double dt)
    {
        float throttle = 0.0f;
        if (eventManager.keyDown[KEY_W])
            throttle = 1.0f;
        else if (eventManager.keyDown[KEY_S])
            throttle = -1.0f;

        accumulator += dt;
        if (accumulator > fixedDt * 8.0)
            accumulator = fixedDt * 8.0;

        while (accumulator >= fixedDt)
        {
            physics.step(fixedDt, throttle);
            accumulator -= fixedDt;
        }

        auto bs = physics.beamStates();
        foreach (i, e; beamEntities)
            if (i < bs.length)
            {
                e.position = bs[i].position;
                e.rotation = bs[i].orientation;
            }

        auto ws = physics.wheelStates();
        foreach (i, e; wheelEntities)
            if (i < ws.length)
            {
                e.position = ws[i].position;
                e.rotation = ws[i].orientation;
            }
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
