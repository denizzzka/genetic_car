module viewer.terrainvisualizer;

import std.math;

import dlib.core.memory;
import dlib.math.vector;

import dagon;

import frame.frame : origin, frameForward = forward, frameRight = right;
import physics_world;

/// Меш и сущность одного визуального тайла поверхности. Меши живут вне GC
/// (dlib New!), поэтому при выгрузке тайла Delete(mesh) обязателен.
struct VFTile
{
    Mesh mesh;
    Entity entity;
    /// В мире сцены (окно) или выгружен при уходе окна: выгруженный тайл
    /// держится в кэше, и при возврате окна сущность возвращается через
    /// `useEntity` без перестройки меша.
    bool inScene;
}

/**
 * Визуализатор процедурной поверхности во вьюере.
 *
 * Строит меши тайлов из общего shared-кэша (physics_world.terrain) вокруг
 * фокуса (живой машины или origin), по мере езды подвозит новые тайлы и
 * убирает уехавшие. Вершины меша — уже в абсолютных координатах мира dagon
 * (= Newton), поэтому сущности тайлов стоят в origin.
 */
final class TerrainVisualizer
{
    private Scene scene_;
    private TerrainSurface terrain_;
    private Material matTile_;
    private Material matTileDim_;

    /// Окно тайлов вокруг фокуса, как у физики: (windowRadius·2+1)² тайлов.
    private int windowRadius_ = 1;

    /// Живые сущности тайлов окна (уехавшие выгружаются из мира, но не из кэша).
    private VFTile[long] tiles_;
    private enum size_t tileCacheCap = 72;

    this(Scene scene, TerrainSurface terrain)
    {
        scene_ = scene;
        terrain_ = terrain;

        matTile_ = scene.addMaterial();
        matTile_.baseColorFactor = Color4f(1.0f, 1.0f, 1.0f, 1.0f);
        matTile_.roughnessFactor = 1.0f;
        matTile_.metallicFactor = 0.0f;
        matTile_.baseColorTexture = buildHeightGradientTexture();

        // Затемнённый вариант для тайлов вокруг центрального. В dagon при
        // наличии текстуры baseColorFactor (diffuseVector) в шейдере
        // игнорируется — текстура целиком заменяет его, поэтому затемнение
        // делаем внутри самой текстуры: тот же градиент высот, но серенький.
        matTileDim_ = scene.addMaterial();
        matTileDim_.baseColorFactor = Color4f(1.0f, 1.0f, 1.0f, 1.0f);
        matTileDim_.roughnessFactor = matTile_.roughnessFactor;
        matTileDim_.metallicFactor = matTile_.metallicFactor;
        matTileDim_.baseColorTexture = buildDimGradientTexture();
    }

    /// Передвинуть окно за фокусом.
    void update(BuggyPhysics live)
    {
        const vec3 focus = (live is null) ? origin : live.surfaceFocus;
        updateTiles(focus);
    }

    /// Передвинуть окно за точкой фокуса (виртуальная багги: без физики).
    void updateFocus(const vec3 focus)
    {
        updateTiles(focus);
    }

    private void updateTiles(const vec3 focus)
    {
        const TileIndex c = tileIndexAt(focus, terrain_.config);
        const int cx = c.tx;
        const int cy = c.ty;
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
                tiles_[key].inScene = false;
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
            {
                ensureTile(tx, ty);
                // Тайлы вокруг центрального затемняем, центральный остаётся
                // ярким — видно, по какому тайлу едет багги.
                const long key = tileKey(tx, ty);
                tiles_[key].entity.material =
                    (tx == cx && ty == cy) ? matTile_ : matTileDim_;
            }
    }

    private void ensureTile(int tx, int ty)
    {
        const long key = tileKey(tx, ty);
        if (key in tiles_)
        {
            // Тайл уже в кэше, но мог быть выгружен из мира при уходе окна:
            // возвращаем его сущность в сцену, меш не перестраиваем.
            if (!tiles_[key].inScene)
            {
                scene_.useEntity(tiles_[key].entity);
                tiles_[key].inScene = true;
            }
            return;
        }

        immutable cfg = terrain_.config;
        immutable tile = terrain_.tileData(tx, ty);

        auto mesh = buildTileMesh(tile, cfg, tx, ty, cfg.tileSize);
        auto e = scene_.addEntity();
        e.drawable = mesh;
        e.material = matTile_;

        VFTile t;
        t.mesh = mesh;
        t.entity = e;
        t.inScene = true;
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
            + frameForward * (cast(float) tx * tileSize - gridHalfShift(cfg))
            + frameRight * (cast(float) ty * tileSize - gridHalfShift(cfg));

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
                // Сетка высот хранится как [forward·W + right], а вершина
                // этой итерации — (forward = kx, right = ky).
                const float h = tile.heights[kx * W + ky];
                const vec3 p = corner
                    + frameForward * (cast(float) kx * cell)
                    + frameRight * (cast(float) ky * cell);
                // Точка угла тайла на плоскости, затем высота вверх (Newton +Y).
                const vec3 v = toNewtonPos(p) + vec3(0.0f, h, 0.0f);
                mesh.vertices[i] = v;

                // Нормаль центральными разностями по сетке высот: вдоль
                // forward меняется старший индекс, вдоль right — младший.
                const float hFw = (kx + 1 < W) ? tile.heights[(kx + 1) * W + ky]
                    : tile.heights[(kx - 1) * W + ky];
                const float hBk = (kx > 0) ? tile.heights[(kx - 1) * W + ky]
                    : tile.heights[(kx + 1) * W + ky];
                const float hRt = (ky + 1 < W) ? tile.heights[kx * W + ky + 1]
                    : tile.heights[kx * W + ky - 1];
                const float hLf = (ky > 0) ? tile.heights[kx * W + ky - 1]
                    : tile.heights[kx * W + ky + 1];
                const float gu = (hFw - hBk) / (2.0f * cell);
                const float gv = (hRt - hLf) / (2.0f * cell);
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
        return buildGradientTexture(1.0f);
    }

    /// Серенький вариант той же текстуры: тот же градиент высот, но цвета
    /// смешаны с нейтральным серым — «подкрашенные» тайлы вокруг центрального.
    private Texture buildDimGradientTexture()
    {
        return buildGradientTexture(0.0f);
    }

    private Texture buildGradientTexture(const float grayMix)
    {
        const int imgW = 4;
        const int imgH = 64;
        const Color4f low = Color4f(0.30f, 0.48f, 0.22f, 1.0f);
        const Color4f mid = Color4f(0.52f, 0.46f, 0.34f, 1.0f);
        const Color4f high = Color4f(0.60f, 0.58f, 0.56f, 1.0f);
        const Color4f gray = Color4f(0.45f, 0.45f, 0.45f, 1.0f);

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
            {
                const Color4f d = Color4f(
                    c.r * grayMix + gray.r * (1.0f - grayMix),
                    c.g * grayMix + gray.g * (1.0f - grayMix),
                    c.b * grayMix + gray.b * (1.0f - grayMix),
                    1.0f);
                img[x, y] = d;
            }
        }

        auto tex = New!Texture(scene_);
        tex.createFromImage(img, false);
        Delete(img);
        return tex;
    }
}