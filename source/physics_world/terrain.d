module physics_world.terrain;

import std.math;
import std.random : Mt19937, uniform;
import std.algorithm : clamp;
import core.sync.mutex : Mutex;

import fast_noise;
import dlib.math.vector;

import frame.frame;

/*
 * Процедурная поверхность живёт в базисе каркаса (см. frame.frame): плоскость
 * земли — это плоскость вдоль `forward` и `right` через `origin`, высота — по
 * `up`. Тайл (tx, ty) — квадрат на этой плоскости со стороной `tileSize`,
 * покрывающий
 *
 *     origin + forward·(tx·S) + right·(ty·S)  +  (forward·u + right·v), u,v ∈ [0,S)
 */

/// Параметры процедурной поверхности. Неизменяемые копии спокойно шарит
/// shared-кэш между фитнес-заездами и вьюером.
struct TerrainConfig
{
    /// Зерно всех генераторов шума и булыжников: тайл полностью детерминирован.
    int seed = 1337;

    /// Размер стороны тайла в метрах (по обеим осям плоскости forward×right).
    float tileSize = 8.0f;

    /// Число ячеек тайла на сторону: grid W = cells+1 вершин на сторону,
    /// шаг cell = tileSize / cells.
    uint cells = 64;

    /// Радиус плавного наката сложности от старта: за `rampLength` метров
    /// (по hypot вдоль плоскости) амплитуда и частота неровностей доходят до
    /// максимума через smoothstep(0, rampLength, d).
    float rampLength = 120.0f;

    /// Амплитуда неровностей у старта (почти ровно) и вдали от него, м.
    float baseAmplitude = 0.05f;
    float maxAmplitude = 1.4f;

    /// Частота главного шума у старта и вдали.
    float baseFrequency = 0.08f;
    float maxFrequency = 0.45f;

    /// Мелкая детализация, добавляется только на накатанной части (× t).
    float detailAmplitude = 0.9f;
    float detailFrequency = 0.12f;

    uint mainOctaves = 4;
    uint detailOctaves = 5;
    float lacunarity = 2.0f;
    float gain = 0.5f;

    /// Радиусы булыжников: у старта они минимальны и накатываются до
    /// максимума по мере удаления (тот же rampLength, что у рельефа).
    float boulderRadiusMin = 0.25f;
    float boulderRadiusMax = 0.5f;
    uint maxBouldersPerTile = 8;

    /// Не сеять булыжники ближе этой дистанции к origin — старт чистый.
    float spawnClearRadius = 3.0f;

    /// Верхняя граница числа тайлов в кэше; при переполнении кэш чистится
    /// целиком. Окно стриминга 5×5 — реально в кэше порядка десятка тайлов.
    size_t cacheCap = 2048;
}

/// Булыжник: круглое препятствие на местности. Положение — смещение от угла
/// тайла вдоль `forward` и `right`.
struct BoulderData
{
    float alongForward;
    float alongRight;
    float radius;
}

/// Данные тайла: сетка высот и булыжники. Обычный GC-класс; публикуется
/// из кэша как immutable и после этого никогда не мутируется.
class TerrainTileData
{
    /// W·W высот вдоль `up`. Индекс `[ky·W + kx]`: kx — столбец вдоль
    /// `forward` (от угла тайла), ky — ряд вдоль `right`. Порядок совпадает
    /// с буфером heightfield Newton 1:1 (см. terrainworld.d).
    float[] heights;

    /// Булыжники тайла.
    BoulderData[] boulders;
}

/// Единая аналитическая поверхность: для точки `p` на плоскости каркаса
/// (т.е. начало координат + компоненты вдоль `forward` и `right`) возвращает
/// высоту вдоль `up`. Плавный накат сложности по дистанции от origin. Её же
/// считает кэш тайлов, поэтому смежные тайлы бесшовны по построению.
float terrainHeightAt(const TerrainConfig cfg, const vec3 p)
{
    const vec3 uDir = forward;
    const vec3 vDir = right;
    const float u = dot(p, uDir); // компонента вдоль forward
    const float v = dot(p, vDir); // компонента вдоль right
    const float d = hypot(u, v);
    const float t = smoothstep01(d / cfg.rampLength);
    const float amp = mix(cfg.baseAmplitude, cfg.maxAmplitude, t);
    const float freq = mix(cfg.baseFrequency, cfg.maxFrequency, t);

    float h = fbm01(cfg.seed, u, v, freq, cfg.mainOctaves) * amp;
    h += fbm01(cfg.seed + 100, u, v, cfg.detailFrequency, cfg.detailOctaves)
        * cfg.detailAmplitude * t;
    return h;
}

/// FBM-шум OpenSimplex2 в [-1,1] в точке (u, v). Состояние FastNoise строится
/// на каждый вызов и только читается внутри — потокобезопасно. lacunarity и
/// gain фиксированы константами поверхности.
private float fbm01(int seed, float u, float v, float frequency, uint octaves)
{
    FNLState s = fnlCreateState(seed);
    s.fractal_type = FNLFractalType.FNL_FRACTAL_FBM;
    s.frequency = frequency;
    s.octaves = cast(int) octaves;
    s.lacunarity = 2.0f;
    s.gain = 0.5f;
    return cast(float) fnlGetNoise2D(&s, u, v);
}

/// Ключ тайла: (tx, ty) упакованы в long без коллизий.
long tileKey(int tx, int ty)
{
    return (cast(long) cast(uint) tx << 32) | cast(uint) ty;
}

/// Координаты тайла из ключа.
int tileX(long key)
{
    return cast(int) (key >> 32);
}

int tileY(long key)
{
    return cast(int) (key & 0xFFFFFFFF);
}

/// Детерминированный сид генератора булыжников тайла.
private ulong tileSeed(const TerrainConfig cfg, int tx, int ty)
{
    ulong h = cast(ulong) cfg.seed;
    h = h * 0x100000001b3 + cast(uint) tx;
    h = h * 0x100000001b3 + cast(uint) ty;
    return h;
}

private BoulderData[] buildBoulders(const TerrainConfig cfg, int tx, int ty)
{
    auto gen = Mt19937(cast(uint) tileSeed(cfg, tx, ty));
    const vec3 corner = origin
        + forward * (cast(float) tx * cfg.tileSize)
        + right * (cast(float) ty * cfg.tileSize);
    const vec3 tileCenter = corner
        + forward * (cfg.tileSize * 0.5f)
        + right * (cfg.tileSize * 0.5f);
    // Диапазон радиусов: мелкие камни есть и у старта, крупные добавляются
    // по мере удаления (тот же накат rampLength). Плюс рандом внутри
    // диапазона — камни не одинаковые даже на одном удалении.
    const float d = hypot(dot(tileCenter - origin, forward), dot(tileCenter - origin, right));
    const float t = smoothstep01(d / cfg.rampLength);
    const float rLo = cfg.boulderRadiusMin * 0.5f;
    const float rHi = mix(cfg.boulderRadiusMin, cfg.boulderRadiusMax, t);
    BoulderData[] res;
    foreach (_; 0 .. cfg.maxBouldersPerTile)
    {
        const float lf = uniform(0.0f, cfg.tileSize, gen);
        const float lr = uniform(0.0f, cfg.tileSize, gen);
        const vec3 c = corner + forward * lf + right * lr;
        if ((c.length) < cfg.spawnClearRadius)
            continue;
        BoulderData b;
        b.alongForward = lf;
        b.alongRight = lr;
        b.radius = uniform(rLo, rHi, gen);
        res ~= b;
    }
    return res;
}

/// Shared-кэш процедурной земли: держатель тайлов, общий для всех заездов
/// фитнес-пула и вьюера. Каждый тайл строится один раз и отдаётся как
/// immutable; построение идёт под мьютексом.
shared class TerrainSurface
{
    private immutable TerrainConfig cfg_;
    private shared Mutex lock_;
    private shared(TerrainTileData)[long] tiles_;

    this(immutable TerrainConfig cfg)
    {
        cfg_ = cfg;
        lock_ = cast(shared) new Mutex();
        tiles_ = null;
    }

    /// Копия неизменяемых параметров поверхности.
    TerrainConfig config() shared
    {
        return cfg_;
    }

    /// Тайл (tx, ty): построить при первом запросе, дальше отдавать из кэша.
    immutable(TerrainTileData) tileData(int tx, int ty) shared
    {
        const long key = tileKey(tx, ty);
        synchronized (lock_)
        {
            auto it = key in tiles_;
            if (it !is null)
                return cast(immutable) *it;

            const uint W = cfg_.cells + 1;
            const float cell = cfg_.tileSize / cast(float) cfg_.cells;
            const vec3 corner = origin
                + forward * (cast(float) tx * cfg_.tileSize)
                + right * (cast(float) ty * cfg_.tileSize);

            auto t = new TerrainTileData;
            t.heights = new float[W * W];
            foreach (ky; 0 .. W)
                foreach (kx; 0 .. W)
                {
                    const vec3 p = corner
                        + forward * (cast(float) kx * cell)
                        + right * (cast(float) ky * cell);
                    t.heights[ky * W + kx] = terrainHeightAt(cfg_, p);
                }
            t.boulders = buildBoulders(cfg_, tx, ty);

            tiles_[key] = cast(shared) t;
            if (tiles_.length > cfg_.cacheCap)
                tiles_ = null;
            return cast(immutable) t;
        }
    }

    /// Высота поверхности в точке каркаса (аналитически, без кэша).
    float heightAt(const vec3 p) shared
    {
        return terrainHeightAt(cfg_, p);
    }
}

/// Кэш земли по умолчанию. Инициализируется под __gshared-локом по образцу
/// ensureNewtonLoaded: первый же дозвавшийся поток строит общий экземпляр.
private __gshared Object terrainLock_ = new Object();
private __gshared Object terrainHolder_;

/// Единственный общий кэш поверхности на процесс: фитнес и вьюер читают одни
/// и те же тайлы (immutable), физическое стримингование — на стороне каждого
/// мира.
TerrainSurface sharedTerrain()
{
    if (terrainHolder_ !is null)
        return cast(TerrainSurface) terrainHolder_;
    synchronized (terrainLock_)
    {
        if (terrainHolder_ is null)
            terrainHolder_ = cast(Object) new TerrainSurface(TerrainConfig.init);
        return cast(TerrainSurface) terrainHolder_;
    }
}

unittest
{
    // Поверхность детерминирована и плавно стартует с ровной земли.
    auto t = sharedTerrain();
    const float h0 = t.heightAt(origin);
    assert(abs(h0) < 0.5f, "в origin поверхность почти ровная");
    const vec3 p = forward * 10.0f - right * 3.0f;
    assert(t.heightAt(p) == t.heightAt(p),
        "высота детерминирована");

    // Тайлы кэшируются: второй запрос — тот же immutable, без пересчёта.
    const td1 = t.tileData(0, 0);
    const td2 = t.tileData(0, 0);
    assert(td1 is td2, "тайл из кэша возвращается тем же объектом");

    // Смежные тайлы бесшовны: общая грань даёт одинаковые высоты.
    const W = t.config.cells + 1;
    const a = t.tileData(0, 0);
    const b = t.tileData(0, 1);
    foreach (kx; 0 .. W)
    {
        const float ha = a.heights[(W - 1) * W + kx];
        const float hb = b.heights[0 * W + kx];
        assert(abs(ha - hb) < 1e-6f, "высоты смежной грани совпадают");
    }
}

private float smoothstep01(float t)
{
    t = clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

private float mix(float a, float b, float t)
{
    return a + (b - a) * t;
}