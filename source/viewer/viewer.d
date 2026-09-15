module viewer.viewer;

import dagon;
import frame.frame;
import frame.buggy;

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

    override void afterLoad()
    {
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

        const half = buggyFrame();
        buildFrame(half);

        auto ePlane = addEntity();
        ePlane.drawable = New!ShapePlane(10.0f, 10.0f, 1, assetManager);
    }

    private void buildFrame(const Frame half)
    {
        const full = mirrorClosure(half);

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
            else if (isOnPlane(a) && isOnPlane(b2))
                mat = matAxial;
            else
                mat = matBeam;

            auto e = addEntity(carRoot);
            e.drawable = New!ShapeCylinder(b.radius, length, 8, assetManager);
            e.material = mat;
            e.position = (a + b2) * 0.5f;
            e.rotation = rotationBetween(Vector3f(0, 1, 0), dir / length);
        }

        foreach (node; half.nodes)
        {
            if (node.kind == AnchorKind.wheel)
            {
                addWheel(node.pos);
                if (!isOnPlane(node.pos))
                    addWheel(mirrorX(node.pos));
            }
            else
            {
                addNodeSphere(node.pos, node.kind);
                if (!isOnPlane(node.pos))
                    addNodeSphere(mirrorX(node.pos), node.kind);
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

    private Material wheelMaterial()
    {
        auto mat = addMaterial();
        mat.baseColorFactor = Color4f(0.08f, 0.08f, 0.08f, 1.0f);
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

