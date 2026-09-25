module physics_world.terrain;

import std.math;
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
    /// Зерно генераторов шума: тайл полностью детерминирован.
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

    /// Верхняя граница числа тайлов в кэше; при переполнении кэш чистится
    /// целиком. Окно стриминга 5×5 — реально в кэше порядка десятка тайлов.
    size_t cacheCap = 2048;
}

/// Данные тайла: сетка высот. Обычный GC-класс; публикуется из кэша как
/// immutable и после этого никогда не мутируется.
class TerrainTileData
{
    /// W·W высот вдоль `up`. Индекс `[f·W + r]`: f — ряд вдоль `forward`
    /// (от угла тайла), r — столбец вдоль `right`. Это ровно порядок
    /// heightfield Newton (`[zRow·W + xCol]`, z = локальный forward,
    /// x = локальный right), поэтому физика забирает высоты 1:1, без поворота.
    float[] heights;
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

/// Полтайла: сдвиг сетки тайлов на полтайла по обеим осям, чтобы стартовый
/// origin лежал в центре тайла, а не на стыке четырёх. На стыке физическое
/// высотное поле теряет контакт с колесом, и машина проваливается.
float gridHalfShift(const TerrainConfig cfg) pure nothrow @nogc
{
    return 0.5f * cfg.tileSize;
}

/// Номер тайла, под которым лежит точка фокуса (плоскость каркаса).
///
/// Фокус задан в координатах каркаса: `focus.x` — смещение вдоль `right`,
/// `focus.y = −forward` (у каркаса forward = (0,−1,0)). Корнер тайла (tx, ty):
/// `corner.x = ty·S − S/2` (right), `corner.y = −(tx·S − S/2)` (forward), т.е.
/// центр тайла tx отвечает forward = tx·S, центр taйла ty — right = ty·S.
TileIndex tileIndexAt(const vec3 focus, const TerrainConfig cfg)
    pure nothrow @nogc
{
    const float S = cfg.tileSize;
    return TileIndex(cast(int) floor(-focus.y / S + 0.5f),
        cast(int) floor(focus.x / S + 0.5f));
}

/// Индексы тайла в сетке: tx — вдоль forward, ty — вдоль right.
struct TileIndex
{
    int tx, ty;
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
                + forward * (cast(float) tx * cfg_.tileSize - gridHalfShift(cfg_))
                + right * (cast(float) ty * cfg_.tileSize - gridHalfShift(cfg_));

            auto t = new TerrainTileData;
            t.heights = new float[W * W];
            foreach (f; 0 .. W)
                foreach (r; 0 .. W)
                {
                    const vec3 p = corner
                        + forward * (cast(float) f * cell)
                        + right * (cast(float) r * cell);
                    t.heights[f * W + r] = terrainHeightAt(cfg_, p);
                }

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
    // Индексы тайла из фокуса: ось right (focus.x) даёт индекс ty, ось
    // forward (focus.y = −forward) — индекс tx. Именно в такой логике дальше
    // работают и физика окна, и визуализатор.
    auto t = sharedTerrain();
    const cfg = t.config;
    const float S = cfg.tileSize;

    // Фокус по центру тайла (tx=0, ty=1): right = 1,2·S, forward = 0,2·S.
    // В фокусе forward-координата уходит в y со знаком: focus.y = −forward.
    const vec3 focus = right * (1.2f * S) + forward * (0.2f * S);
    const TileIndex c = tileIndexAt(focus, cfg);
    assert(c.tx == 0 && c.ty == 1,
        "право/вперёд: tx — forward-индекс, ty — right-индекс");

    const TileIndex c2 = tileIndexAt(origin, cfg);
    assert(c2.tx == 0 && c2.ty == 0, "origin — центр тайла (0, 0)");
}

unittest
{
    // Поверхность детерминирована и плавно стартует с ровной земли.
    auto t = sharedTerrain();
    const float h0 = t.heightAt(origin);
    assert(abs(h0) < 0.5f, "в origin поверхность почти ровная");
    const vec3 p = forward * 10.0f - right * 3.0f;
    // Детерминизм: поверхность одной и той же точки, спрошенная дважды,
    // отдаёт одинаковую высоту (нет скрытого рандома в генерации).
    const float h1 = t.heightAt(p);
    const float h2 = t.heightAt(p);
    assert(h1 == h2,
        "два запроса высоты одной точки дают одинаковую величину");

    // Тайлы кэшируются: второй запрос — тот же immutable, без пересчёта.
    const td1 = t.tileData(0, 0);
    const td2 = t.tileData(0, 0);
    assert(td1 is td2, "тайл из кэша возвращается тем же объектом");

    // Смежные тайлы бесшовны: общая грань даёт одинаковые высоты.
    const W = t.config.cells + 1;
    const a = t.tileData(0, 0);
    const b = t.tileData(0, 1);
    foreach (f; 0 .. W)
    {
        const float ha = a.heights[f * W + (W - 1)];
        const float hb = b.heights[f * W + 0];
        assert(abs(ha - hb) < 1e-6f, "высоты смежной грани совпадают");
    }
}

unittest
{
    // Порядок хранения совпадает с heightfield Newton: heights[f·W + r]
    // отвечает физической точке (forward = f, right = r). Тогда физика
    // забирает буфер 1:1, без транспонирования, и стыки плиток остаются
    // бесшовными (грань одного тайла — та же аналитическая кривая, что у
    // соседа).
    auto t = sharedTerrain();
    const cfg = t.config;
    const uint W = cfg.cells + 1;
    const float cell = cfg.tileSize / cast(float) cfg.cells;
    const tile = t.tileData(0, 0);

    const vec3 corner = origin + forward * (-gridHalfShift(cfg))
        + right * (-gridHalfShift(cfg));
    foreach (f; 0 .. W)
        foreach (r; 0 .. W)
        {
            const vec3 p = corner + forward * (cast(float) f * cell)
                + right * (cast(float) r * cell);
            assert(abs(tile.heights[f * W + r] - terrainHeightAt(cfg, p)) < 1e-6f,
                "heights[f·W + r] отвечает точке (forward = f, right = r)");
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