module viewer.viewer;

import dagon;
import dagon.core.keycodes;
import dagon.core.time;
import std.random;
import car.car;
import frame.frame;
import frame.buggy;
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

    Grammar grammar;
    Random rnd;
    Buggy current;

    override void afterLoad()
    {
        eventManager.trackUpDownState = true;
        grammar = buggyGrammar();
        rnd = Random(unpredictableSeed);

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

        auto car = new Buggy(buggyFrame());
        buildCar(car);
        current = car;

        auto ePlane = addEntity();
        ePlane.drawable = New!ShapePlane(10.0f, 10.0f, 1, assetManager);
    }

    override void update(Time t)
    {
        super.update(t);

        if (eventManager.keyDown[KEY_R])
        {
            removeCar();
            current = new Buggy(buggyFrame());
            buildCar(current);
        }
        else if (eventManager.keyDown[KEY_M])
        {
            auto g = randomGenotype(grammar, 8, rnd);
            mutate(g, 0.05f, rnd);

            bool ok;
            auto f = develop(grammar, g, ok);
            if (ok)
            {
                removeCar();
                current = new Buggy(f);
                buildCar(current);
            }
        }
    }

    private void removeCar()
    {
        foreach (e; carRoot.children)
            removeEntity(e);
    }

    private void buildCar(const Buggy car)
    {
        const full = car.full;

        auto matBeam = addMaterial();
        matBeam.baseColorFactor = Color4f(0.55f, 0.55f, 0.62f, 1.0f);
        matBeam.metallicFactor = 0.7f;

        auto matCross = addMaterial();
        matCross.baseColorFactor = Color4f(0.9f, 0.85f, 0.15f, 1.0f);
        matCross.metallicFactor = 0.7f;

        auto matAxial = addMaterial();
        matAxial.baseColorFactor = Color4f(0.2f, 0.8f, 0.25f, 1.0f);
        matAxial.metallicFactor = 0.7f;

        foreach (b; full.beams)
        {
            const a = full.nodes[b.a];
            const b2 = full.nodes[b.b];
            const dir = b2 - a;
            const float length = dir.length;
            if (length < 1e-5f)
                continue;

            Material mat;
            if (b.kind == BeamKind.cross)
                mat = matCross;
            else if (b.kind == BeamKind.axial)
                mat = matAxial;
            else
                mat = matBeam;

            auto e = addEntity(carRoot);
            e.drawable = New!ShapeCylinder(b.radius, length, 8, assetManager);
            e.material = mat;
            e.position = (a + b2) * 0.5f;
            e.rotation = rotationBetween(Vector3f(0, 1, 0), dir / length);
        }

        foreach (i, nodePos; full.nodes)
        {
            switch (car.kinds[i])
            {
                case AnchorKind.wheel:
                    addWheel(nodePos);
                    break;
                case AnchorKind.wheelDrive:
                    addDriveWheel(nodePos);
                    break;
                default:
                    addNodeSphere(nodePos, car.kinds[i]);
                    break;
            }
        }
    }

    private void addWheel(const vec3 pos)
    {
        auto e = addEntity(carRoot);
        e.drawable = New!ShapeTorus(0.2f, 0.1f, 16, 8, assetManager);
        e.material = wheelMaterial();
        e.position = pos;
        e.rotation = rotationBetween(Vector3f(0, 1, 0), Vector3f(1, 0, 0));
    }

    private void addDriveWheel(const vec3 pos)
    {
        auto e = addEntity(carRoot);
        e.drawable = New!ShapeTorus(0.2f, 0.1f, 16, 8, assetManager);
        e.material = driveWheelMaterial();
        e.position = pos;
        e.rotation = rotationBetween(Vector3f(0, 1, 0), Vector3f(1, 0, 0));
    }

    private Material wheelMaterial()
    {
        auto mat = addMaterial();
        mat.baseColorFactor = Color4f(0.08f, 0.08f, 0.08f, 1.0f);
        mat.roughnessFactor = 0.9f;
        mat.metallicFactor = 0.0f;
        return mat;
    }

    private Material driveWheelMaterial()
    {
        auto mat = addMaterial();
        mat.baseColorFactor = Color4f(0.6f, 0.1f, 0.1f, 1.0f);
        mat.roughnessFactor = 0.9f;
        mat.metallicFactor = 0.0f;
        return mat;
    }

    private void addNodeSphere(const vec3 pos, AnchorKind kind)
    {
        auto e = addEntity(carRoot);
        e.drawable = New!ShapeSphere(0.03f, assetManager);
        e.material = kindMaterial(kind);
        e.position = pos;
    }

    private Material kindMaterial(AnchorKind kind)
    {
        auto mat = addMaterial();
        mat.metallicFactor = 0.3f;
        final switch (kind)
        {
            case AnchorKind.wheel:
            case AnchorKind.wheelDrive:
                mat.baseColorFactor = Color4f(0.05f, 0.05f, 0.05f, 1.0f);
                break;
            case AnchorKind.motor:
                mat.baseColorFactor = Color4f(0.9f, 0.15f, 0.1f, 1.0f);
                break;
            case AnchorKind.shock:
                mat.baseColorFactor = Color4f(0.1f, 0.5f, 0.9f, 1.0f);
                break;
            case AnchorKind.spring:
                mat.baseColorFactor = Color4f(1.0f, 0.6f, 0.1f, 1.0f);
                break;
            case AnchorKind.axle:
                mat.baseColorFactor = Color4f(0.6f, 0.2f, 0.8f, 1.0f);
                break;
            case AnchorKind.none:
                mat.baseColorFactor = Color4f(0.45f, 0.45f, 0.45f, 1.0f);
                break;
        }
        return mat;
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

