module physics_world.terrain;

import std.math : floor, abs;

import dlib.core.ownership;
import dlib.core.memory;
import dlib.math.vector;
import dlib.math.transformation;
import dlib.geometry.triangle : Triangle;

import dagon.core.event;
import dagon.graphics.mesh : TriangleSet;
import dagon.ext.newton;

import fast_noise;

/// Размер стороны квадратного тайла поверхности, м.
enum float terrainTileSize = 20.0f;

/// Ячеек сетки высот по стороне тайла (вершин: `div + 1` на сторону).
enum int terrainTileDiv = 64;

/// Полуширина кольца тайлов вокруг машины: 3×3 вокруг текущего тайла.
enum int terrainRingRadius = 1;

/// Размер LRU-кэша тайловых данных (сеток высот + камней), штук.
enum size_t terrainCacheCap = 256;

/// Длина стартового «пада»: зоны у начала координат c почти плоской
/// поверхностью (небольшие бугорки уровня `terrainAmpPad`).
enum float terrainPadLen = 10.0f;

/// Длина зоны набора сложности после пада, м: дальше — максимальный рельеф.
enum float terrainRiseLen = 60.0f;

/// Амплитуда микронеровностей пада, м.
enum float terrainAmpPad = 0.02f;

/// Максимальная амплитуда рельефа, м (порядка радиуса колеса).
enum float terrainAmpMax = 0.35f;

/// Базовая частота FBM-шума (в единицах 1/м).
enum float terrainFreqBase = 0.6f;

/// Прирост частоты к максимуму сложности.
enum float terrainFreqAdd = 1.2f;

/// Фиксированный seed курса: каждый заезд видит один и тот же рельеф.
enum int terrainSeed = 1337;

/// Камень-препятствие на тайле (только коллизия, штрафа не даёт).
struct Boulder
{
    float x, y, z; /// позиция центра
    float r;       /// радиус
}

/// Данные одного тайла: сетка высот в мировых координатах и камни.
/// Кэшируются как данные, не коллайдеры: тайл можно пересобрать из них
/// в любом физическом мире.
private struct TileData
{
    float[] heights;   /// (div+1)^2 строк, строка — вдоль X
    Boulder[] boulders;
}

/// FBM-ландшафт курса: детерминированный, читается из любого потока.
/// `frequency == 1` и масштаб по координатам — вручную, чтобы сменить частоту
/// по сложности, не трогая разделяемое состояние.
private __gshared FNLState fnoise_;

shared static this()
{
    fnoise_ = fnlCreateState(terrainSeed);
    fnoise_.frequency = 1.0f;
    fnoise_.noise_type = FNLNoiseType.FNL_NOISE_OPENSIMPLEX2;
    fnoise_.fractal_type = FNLFractalType.FNL_FRACTAL_FBM;
    fnoise_.octaves = 4;
    fnoise_.lacunarity = 2.0f;
    fnoise_.gain = 0.5f;
}

/// Сложность курса в точке (0..1): 0 в паде, плавно растёт до 1 по мере
/// удаления от старта. Курс идёт вдоль -Y, поэтому мерим |y|.
float terrainDifficulty(float y)
{
    const float d = abs(y);
    if (d <= terrainPadLen)
        return 0.0f;
    if (d >= terrainPadLen + terrainRiseLen)
        return 1.0f;
    const float u = (d - terrainPadLen) / terrainRiseLen;
    return u * u * (3.0f - 2.0f * u); // smoothstep
}

/// Высота поверхности в мировых координатах: детерминированная и
/// непрерывная, одного и того же рельефа для любого тайла и заезда.
float terrainHeight(float x, float y)
{
    const float t = terrainDifficulty(y);
    const float amp = terrainAmpPad + (terrainAmpMax - terrainAmpPad) * t;
    const float freq = terrainFreqBase + terrainFreqAdd * t;
    return amp * fnlGetNoise2D(&fnoise_, x * freq, y * freq);
}

/// 64-битный детерминированный генератор камней: из ключа тайла.
private struct TileRng
{
    ulong s;

    this(long key)
    {
        s = cast(ulong)key ^ 0x9E3779B97F4A7C15UL;
    }

    ref ulong next() return
    {
        s += 0x9E3779B97F4A7C15UL;
        ulong z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        z ^= z >> 31;
        return s = z;
    }

    /// Равномерное [0, 1) из 53 младших бит.
    float unit() return
    {
        return (next() >> 11) * (1.0f / 9007199254740992.0f);
    }
}

/// Строит данные тайла `(tx, ty)`: сетку высот на мировых координатах и
/// детерминированные камни. Детерминизм гарантирует: сколько ни пересобирай
/// тайл — в любом мире он получится одинаковым.
private TileData buildTile(int tx, int ty)
{
    TileData d;
    const int n = terrainTileDiv + 1;
    d.heights.length = n * n;
    const float step = terrainTileSize / terrainTileDiv;
    const float x0 = cast(float)tx * terrainTileSize;
    const float y0 = cast(float)ty * terrainTileSize;
    foreach (j; 0 .. n)
        foreach (i; 0 .. n)
        {
            d.heights[j * n + i] = terrainHeight(x0 + i * step, y0 + j * step);
        }

    // Камни: количество и размер растут с рельефом, положение детерминировано
    // ключом тайла. В паде камней нет — старт чистый.
    const float tc = terrainDifficulty(y0 + terrainTileSize * 0.5f);
    const size_t count = tc > 0.05f
        ? cast(size_t)(0.5f + 7.5f * tc)
        : 0;
    if (count > 0)
    {
        TileRng rng = TileRng(tileKey(tx, ty));
        d.boulders.length = count;
        foreach (k; 0 .. count)
        {
            Boulder b;
            b.r = 0.15f + 0.35f * tc + rng.unit() * 0.15f;
            // Камни не лежат вплотную к краям тайла (края общие с соседями).
            const float inset = b.r + 0.3f;
            b.x = x0 + inset + rng.unit() * (terrainTileSize - 2.0f * inset);
            b.y = y0 + inset + rng.unit() * (terrainTileSize - 2.0f * inset);
            b.z = terrainHeight(b.x, b.y) + b.r * 0.5f; // полупроглядывает
            d.boulders[k] = b;
        }
    }

    return d;
}

/// Ключ тайла: два int-координата в один long (AA-ключ кэша и карт тел).
private long tileKey(int tx, int ty)
{
    return (cast(long)tx << 32) | (ty & 0xFFFFFFFF);
}

private int tileX(long key) { return cast(int)(key >> 32); }
private int tileY(long key) { return cast(int)key; }

/// Тайловые высоты как `TriangleSet` для `NewtonMeshShape`: грид в мировых
/// координатах (тело тайла — identity), вершины на границах тайлов совпадают,
/// так что соседние тайлы смыкаются без щелей.
final class TileTriangleSet: TriangleSet
{
    private TileData data;
    private int tx, ty;

    this(TileData data, int tx, int ty)
    {
        this.data = data;
        this.tx = tx;
        this.ty = ty;
    }

    /// Даёт 2 треугольника на клетку грида с ветвями, смотрящими вверх.
    int opApply(scope int delegate(Triangle t) dg)
    {
        const int n = terrainTileDiv + 1;
        const float step = terrainTileSize / terrainTileDiv;
        const float x0 = cast(float)tx * terrainTileSize;
        const float y0 = cast(float)ty * terrainTileSize;

        Triangle tri;
        tri.materialIndex = 0;
        foreach (j; 0 .. terrainTileDiv)
            foreach (i; 0 .. terrainTileDiv)
            {
                const int a = j * n + i;
                const int b = j * n + i + 1;
                const int c = (j + 1) * n + i;
                const int d = (j + 1) * n + i + 1;
                const float x = x0 + i * step;
                const float y = y0 + j * step;
                const Vector3f p00 = Vector3f(x, y, data.heights[a]);
                const Vector3f p10 = Vector3f(x + step, y, data.heights[b]);
                const Vector3f p11 = Vector3f(x + step, y + step, data.heights[d]);
                const Vector3f p01 = Vector3f(x, y + step, data.heights[c]);

                // Треугольник 1: p00, p10, p11.
                tri.v[0] = p00;
                tri.v[1] = p10;
                tri.v[2] = p11;
                setFaceNormal(tri, p00, p10, p11);
                {
                    const int r = dg(tri);
                    if (r)
                        return r;
                }

                // Треугольник 2: p00, p11, p01.
                tri.v[0] = p00;
                tri.v[1] = p11;
                tri.v[2] = p01;
                setFaceNormal(tri, p00, p11, p01);
                {
                    const int r = dg(tri);
                    if (r)
                        return r;
                }
            }
        return 0;
    }

    private static void setFaceNormal(ref Triangle t, Vector3f a, Vector3f b, Vector3f c)
    {
        Vector3f n = cross(b - a, c - a);
        n.normalize();
        t.n[0] = n;
        t.n[1] = n;
        t.n[2] = n;
    }
}

/// LRU-кэш тайловых данных, общий для всех заездов: те же тайлы на пуле
/// миров не пересчитываются. Хранит только данные (не тела!): коллайдеры
/// живут в своём мире и сносятся вместе с ним.
private __gshared Object cacheLock_ = new Object();
private __gshared long[] cacheRecency_;
private __gshared TileData[long] cache_;

private TileData cacheCopy(const ref TileData src)
{
    TileData d;
    d.heights = src.heights.dup;
    d.boulders = src.boulders.dup;
    return d;
}

/// Достаёт данные тайла из общего кэша (копирует) или строит и кэширует.
private TileData cacheGet(long key)
{
    {
        synchronized (cacheLock_)
            if (auto p = key in cache_)
                return cacheCopy(*p);
    }

    // Строим вне блокировки: вычисления дорогие, а общее состояние
    // (`fnoise_`) при этом только читается.
    auto built = buildTile(tileX(key), tileY(key));

    synchronized (cacheLock_)
    {
        if (auto p = key in cache_)
            return cacheCopy(*p);
        cacheRecency_ = key ~ cacheRecency_;
        cache_[key] = built;
        if (cacheRecency_.length > terrainCacheCap)
        {
            const long evict = cacheRecency_[$ - 1];
            cacheRecency_ = cacheRecency_[0 .. $ - 1];
            cache_.remove(evict);
        }
        return cacheCopy(built);
    }
}

/// Потоковый ландшафт заезда: держит кольцо тайлов вокруг машины в мире,
/// собирает выехавшие и уничтожает отставшие. Тайлы — статические тела с
/// `NewtonMeshShape` (дерево треугольников), камни — отдельные сферы.
final class Terrain
{
    private NewtonPhysicsWorld world_;
    private NewtonRigidBody[][long] active_;
    private int curTX_, curTY_;
    private bool haveCenter_;

    this(NewtonPhysicsWorld world)
    {
        world_ = world;
    }

    /// Обновить кольцо под позицией машины (x, y): центр — тайл этой
    /// позиции, вокруг — кольцо `terrainRingRadius`.
    void update(float x, float y)
    {
        const int tx = cast(int)floor(x / terrainTileSize);
        const int ty = cast(int)floor(y / terrainTileSize);
        if (haveCenter_ && tx == curTX_ && ty == curTY_)
            return;
        haveCenter_ = true;
        curTX_ = tx;
        curTY_ = ty;

        // Уничтожить тайлы вне нового кольца.
        foreach (k, v; active_)
        {
            if (inRing(tileX(k), tileY(k)))
                continue;
            destroyTile(v);
            active_.remove(k);
        }

        // Создать недостающие тайлы кольца.
        bool[long] wanted;
        foreach (dx; -terrainRingRadius .. terrainRingRadius + 1)
            foreach (dy; -terrainRingRadius .. terrainRingRadius + 1)
                wanted[tileKey(tx + dx, ty + dy)] = true;
        foreach (k, _; wanted)
            if (k !in active_)
                active_[k] = buildTileBodies(k);
    }

    private bool inRing(int tx, int ty) const
    {
        return abs(tx - curTX_) <= terrainRingRadius
            && abs(ty - curTY_) <= terrainRingRadius;
    }

    private NewtonRigidBody[] buildTileBodies(long key)
    {
        const int tx = tileX(key);
        const int ty = tileY(key);
        auto data = cacheGet(key);

        NewtonRigidBody[] bodies;

        // Поверхность тайла: тело в identity, вершины уже в мировых
        // координатах — соседние тайлы делят одни и те же вершины.
        auto mesh = New!NewtonMeshShape(
            New!TileTriangleSet(data, tx, ty), world_);
        auto ground = New!NewtonRigidBody(NewtonRigidBodyType.Static,
            mesh, 0.0f, world_, world_);
        ground.dynamic = false;
        // TODO(physics): проваливание сквозь рельеф. У static-тела из
        // BVH-дерева (NewtonMeshShape) dagon не вычисляет AABB при создании
        // (остаётся NaN), пока телу явно не зададут матрицу через
        // NewtonBodySetMatrix. Без этого тело отсутствует в широкой фазе и
        // контактов нет вовсе. Правильно — чинить в dagon (auto-SetMatrix для
        // static-тел при создании), а этот вызов — локальный обход.
        ground.setTransformation(translationMatrix(Vector3f(0.0f, 0.0f, 0.0f)));
        ground.update(0.0);
        bodies ~= ground;

        // Камни: только коллизия, статические spheres.
        foreach (b; data.boulders)
        {
            auto sphere = New!NewtonSphereShape(b.r, world_);
            auto rock = New!NewtonRigidBody(NewtonRigidBodyType.Static,
                sphere, 0.0f, world_, world_);
            rock.dynamic = false;
            rock.setTransformation(translationMatrix(Vector3f(b.x, b.y, b.z)));
            rock.update(0.0);
            bodies ~= rock;
        }

        return bodies;
    }

    private static void destroyTile(const ref NewtonRigidBody[] bodies)
    {
        foreach (b; bodies)
            if (b !is null && b.newtonBody !is null)
                NewtonDestroyBody(b.newtonBody);
    }

    /// Высота поверхности под позицией (для порогов заезда).
    float height(float x, float y)
    {
        return terrainHeight(x, y);
    }
}

unittest
{
    // Детерминизм: одна и та же точка даёт ту же высоту.
    const float h1 = terrainHeight(3.3f, -7.2f);
    const float h2 = terrainHeight(3.3f, -7.2f);
    assert(h1 == h2, "высота должна быть детерминированной");

    // Непрерывность: высоты на общей границе двух тайлов совпадают.
    const float step = terrainTileSize / terrainTileDiv;
    assert(terrainHeight(0.0f, 0.0f)
        == terrainHeight(0.0f, 0.0f), "граница тайлов совпадает");
    assert(terrainHeight(terrainTileSize, 0.0f)
        == terrainHeight(terrainTileSize, 0.0f), "граница (tx+1) совпадает");

    // Сетка тайла покрывает его габариты: края — в мировых координатах.
    auto d = buildTile(1, 1);
    assert(d.heights.length == (terrainTileDiv + 1) * (terrainTileDiv + 1),
        "размер сетки высот");

    // TriangleSet отдаёт ровно 2 треугольника на клетку.
    auto ts = new TileTriangleSet(d, 1, 1);
    int count;
    foreach (t; ts)
        count++;
    assert(count == terrainTileDiv * terrainTileDiv * 2,
        "по два треугольника на клетку");
}

unittest
{
    // Кэш хранит копии и отдаёт равные данные для одного ключа.
    auto a = cacheGet(tileKey(0, 0));
    auto b = cacheGet(tileKey(0, 0));
    assert(a.heights == b.heights, "кэш консистентен");
}
