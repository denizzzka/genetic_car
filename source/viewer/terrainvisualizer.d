module viewer.terrainvisualizer;

import std.math;
import std.algorithm : min, max, clamp;

import dlib.core.memory;
import dlib.math.vector;

import dagon;
import dagon.core.keycodes;

import frame.frame : origin, frameForward = forward, frameRight = right;
import physics_world;

/// Меш и сущность одного визуального тайла поверхности. Меши живут вне GC
/// (dlib New!), поэтому при выгрузке тайла Delete(mesh) обязателен.
struct VFTile
{
    Mesh mesh;
    Entity entity;
}

/// Булыжник: сущность-сфера, общая для окна позиция пересоздаётся каждый
/// кадр по активному списку физики. Кэш по id = (ключ тайла, индекс).
struct VFBoulder
{
    Entity entity;
}

/**
 * Визуализатор процедурной поверхности во вьюере.
 *
 * Строит меши тайлов из общего shared-кэша (physics_world.terrain) вокруг
 * фокуса (живой машины или origin), по мере езды подвозит новые тайлы и
 * убирает уехавшие. Вершины меша — уже в абсолютных координатах мира dagon
 * (= Newton), поэтому сущности тайлов стоят в origin. Булыжники-сферы
 * «переснимаются» из физики живого заезда каждый кадр.
 */
final class TerrainVisualizer
{
    private Scene scene_;
    private TerrainSurface terrain_;
    private Material matTile_;
    private Material matBoulder_;
    private Mesh meshBoulder_;

    /// Окно тайлов вокруг фокуса, как у физики: (windowRadius·2+1)² тайлов.
    private int windowRadius_ = 1;

    /// Живые сущности тайлов окна (уехавшие выгружаются из мира, но не из кэша).
    private VFTile[long] tiles_;
    private enum size_t tileCacheCap = 72;

    /// Кэш сущностей булыжников по id = (ключ тайла, индекс).
    private VFBoulder[ulong] boulders_;
    private enum size_t boulderCacheCap = 256;

    this(Scene scene, TerrainSurface terrain)
    {
        scene_ = scene;
        terrain_ = terrain;

        matTile_ = scene.addMaterial();
        matTile_.baseColorFactor = Color4f(1.0f, 1.0f, 1.0f, 1.0f);
        matTile_.roughnessFactor = 1.0f;
        matTile_.metallicFactor = 0.0f;
        matTile_.baseColorTexture = buildHeightGradientTexture();

        matBoulder_ = scene.addMaterial();
        matBoulder_.baseColorFactor = Color4f(0.45f, 0.38f, 0.3f, 1.0f);
        matBoulder_.roughnessFactor = 0.9f;
        matBoulder_.metallicFactor = 0.0f;

        meshBoulder_ = New!ShapeSphere(1.0f, scene.assetManager);
    }

    /// Передвинуть окно за фокусом и снять с физики булыжники.
    void update(BuggyPhysics live)
    {
        const vec3 focus = (live is null) ? origin : live.surfaceFocus;
        updateTiles(focus);

        if (live is null || live.terrainWorld is null)
            syncBoulders(null);
        else
            syncBoulders(live.terrainWorld.activeBoulders());
    }

    private void updateTiles(const vec3 focus)
    {
        const float S = terrain_.config.tileSize;
        const int cx = cast(int) floor(focus.x / S);
        const int cy = cast(int) floor(focus.y / S);
        const int R = windowRadius_;
        const int tx0 = cx - R, tx1 = cx + R;
        const int ty0 = cy - R, ty1 = cy + R;

        bool insideWindow(long key)
        {
            const int tx = tileX(key), ty = tileY(key);
            return tx >= tx0 && tx <= tx1 && ty >= ty0 && ty <= ty1;
        }

        // Уехавшие тайлы прячем из мира, но держим в кэше: вернётся окно —
        // переиспользуем меш, не строя его заново.
        long[] stale;
        foreach (key; tiles_.keys)
            if (!insideWindow(key))
            {
                scene_.removeEntity(tiles_[key].entity, false);
                stale ~= key;
            }

        // Кэш ограничен: сверх лимита выгруженное освобождаем окончательно.
        while (stale.length && tiles_.length > tileCacheCap)
        {
            const long key = stale[$ - 1];
            stale.length--;
            dropTileCached(key);
        }

        foreach (tx; tx0 .. tx1 + 1)
            foreach (ty; ty0 .. ty1 + 1)
                ensureTile(tx, ty);
    }

    private void ensureTile(int tx, int ty)
    {
        const long key = tileKey(tx, ty);
        if (key in tiles_)
            return;

        immutable cfg = terrain_.config;
        immutable tile = terrain_.tileData(tx, ty);

        auto mesh = buildTileMesh(tile, cfg, tx, ty, cfg.tileSize);
        auto e = scene_.addEntity();
        e.drawable = mesh;
        e.material = matTile_;

        VFTile t;
        t.mesh = mesh;
        t.entity = e;
        tiles_[key] = t;
    }

    /// Окончательно освободить тайл: меш (dlib) Delete, сущность — на GC.
    /// Сущность к этому моменту уже выгружена из мира сценой.
    private void dropTileCached(long key)
    {
        Delete(tiles_[key].mesh);
        tiles_.remove(key);
    }

    /// Cвободная память: снести все сущности и меши. Зовётся при закрытии
    /// сцены/вьюера, чтобы dlib-память мешей не текла впустую.
    void dispose()
    {
        foreach (key; tiles_.keys)
        {
            scene_.removeEntity(tiles_[key].entity, false);
            dropTileCached(key);
        }
        tiles_ = null;

        foreach (id, ref b; boulders_)
            scene_.removeEntity(b.entity, false);
        boulders_ = null;

        Delete(meshBoulder_);
    }

    /// Меш тайла (tx, ty). Вершины — в абсолютных координатах мира dagon
    /// (= Newton): точка угла тайла из точечной формулы `corner` переводится
    /// toNewtonPos, высота поднимается по `up` (в Newton это +Y). Индексы —
    /// пара треугольников на ячейку, обход против часовой стрелки сверху.
    private Mesh buildTileMesh(const TerrainTileData tile, const TerrainConfig cfg,
        int tx, int ty, float tileSize)
    {
        const uint W = cfg.cells + 1;
        const uint n = W * W;
        const float cell = tileSize / cast(float) cfg.cells;

        const vec3 corner = origin
            + frameForward * (cast(float) tx * tileSize)
            + frameRight * (cast(float) ty * tileSize);

        const float hLow = -0.5f;
        const float hRange = 4.5f;

        auto mesh = New!Mesh(scene_.assetManager);
        mesh.vertices = New!(Vector3f[])(n);
        mesh.normals = New!(Vector3f[])(n);
        mesh.texcoords = New!(Vector2f[])(n);

        foreach (ky; 0 .. W)
        {
            foreach (kx; 0 .. W)
            {
                const size_t i = ky * W + kx;
                const float h = tile.heights[i];
                const vec3 p = corner
                    + frameForward * (cast(float) kx * cell)
                    + frameRight * (cast(float) ky * cell);
                // Точка угла тайла на плоскости, затем высота вверх (Newton +Y).
                const vec3 v = toNewtonPos(p) + vec3(0.0f, h, 0.0f);
                mesh.vertices[i] = v;

                // Нормаль центральными разностями по сетке высот.
                const float hR = (kx + 1 < W) ? tile.heights[ky * W + kx + 1]
                    : tile.heights[ky * W + kx - 1];
                const float hL = (kx > 0) ? tile.heights[ky * W + kx - 1]
                    : tile.heights[ky * W + kx + 1];
                const float hF = (ky + 1 < W) ? tile.heights[(ky + 1) * W + kx]
                    : tile.heights[(ky - 1) * W + kx];
                const float hB = (ky > 0) ? tile.heights[(ky - 1) * W + kx]
                    : tile.heights[(ky + 1) * W + kx];
                const float gu = (hR - hL) / (2.0f * cell);
                const float gv = (hF - hB) / (2.0f * cell);
                mesh.normals[i] = Vector3f(-gu, 1.0f, gv).normalized;

                mesh.texcoords[i] = Vector2f(
                    cast(float) kx / cast(float)(W - 1),
                    1.0f - clamp((h - hLow) / hRange, 0.0f, 1.0f));
            }
        }

        const ulong quads = (W - 1) * (W - 1);
        mesh.indices = New!(uint[3][])(2 * quads);
        ulong k = 0;
        foreach (ky; 0 .. W - 1)
        {
            foreach (kx; 0 .. W - 1)
            {
                const uint i00 = cast(uint)(ky * W + kx);
                const uint i01 = cast(uint)(ky * W + kx + 1);
                const uint i10 = cast(uint)((ky + 1) * W + kx);
                const uint i11 = cast(uint)((ky + 1) * W + kx + 1);
                mesh.indices[k++] = [i00, i11, i10];
                mesh.indices[k++] = [i00, i01, i11];
            }
        }

        mesh.dataReady = true;
        mesh.calcBoundingBox();
        mesh.prepareVAO();
        return mesh;
    }

    /// Текстура-градиент высот: низ — зелень, выше — мягкая скала.
    private Texture buildHeightGradientTexture()
    {
        const int imgW = 4;
        const int imgH = 64;
        const Color4f low = Color4f(0.30f, 0.48f, 0.22f, 1.0f);
        const Color4f mid = Color4f(0.52f, 0.46f, 0.34f, 1.0f);
        const Color4f high = Color4f(0.60f, 0.58f, 0.56f, 1.0f);

        SuperImage img = unmanagedImage(imgW, imgH, 4, 8);
        foreach (y; 0 .. imgH)
        {
            const float t = cast(float)y / cast(float)(imgH - 1);
            Color4f c;
            if (t < 0.5f)
            {
                const float q = t / 0.5f;
                c = Color4f(
                    low.r + (mid.r - low.r) * q,
                    low.g + (mid.g - low.g) * q,
                    low.b + (mid.b - low.b) * q,
                    1.0f);
            }
            else
            {
                const float q = (t - 0.5f) / 0.5f;
                c = Color4f(
                    mid.r + (high.r - mid.r) * q,
                    mid.g + (high.g - mid.g) * q,
                    mid.b + (high.b - mid.b) * q,
                    1.0f);
            }
            foreach (x; 0 .. imgW)
                img[x, y] = c;
        }

        auto tex = New!Texture(scene_);
        tex.createFromImage(img, false);
        Delete(img);
        return tex;
    }

    /// id булыжника: ключ тайла в старших битах, индекс — в младших.
    private static ulong boulderId(long key, size_t index)
    {
        return (cast(ulong) key << 20) | cast(ulong)(index & 0xFFFFF);
    }

    private void syncBoulders(LiveBoulder[] active)
    {
        ulong[] present;
        if (active !is null)
            foreach (lb; active)
            {
                const ulong id = boulderId(lb.key, lb.index);
                present ~= id;
                auto it = id in boulders_;
                if (it is null)
                {
                    if (boulders_.length > boulderCacheCap)
                        continue;
                    auto e = scene_.addEntity();
                    e.drawable = meshBoulder_;
                    e.material = matBoulder_;
                    VFBoulder b;
                    b.entity = e;
                    boulders_[id] = b;
                    it = id in boulders_;
                }
                else
                {
                    scene_.useEntity(it.entity, false);
                }
                it.entity.position = lb.position;
                it.entity.scaling = Vector3f(lb.radius, lb.radius, lb.radius);
            }

        // Исчезнувшие (уехавшие из окна) — прячем, но держим в кэше.
        foreach (id, ref b; boulders_)
        {
            bool found = false;
            foreach (p; present)
                if (p == id)
                {
                    found = true;
                    break;
                }
            if (!found)
                scene_.removeEntity(b.entity, false);
        }

        if (boulders_.length > boulderCacheCap)
        {
            // Держим только живые, остальное освобождаем окончательно.
            ulong[] evict;
            foreach (id, ref b; boulders_)
            {
                bool found = false;
                foreach (p; present)
                    if (p == id)
                    {
                        found = true;
                        break;
                    }
                if (!found)
                    evict ~= id;
            }
            foreach (id; evict)
            {
                scene_.removeEntity(boulders_[id].entity, false);
                boulders_.remove(id);
            }
        }
    }
}