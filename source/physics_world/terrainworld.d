module physics_world.terrainworld;

import std.math;

import dlib.core.memory;
import dlib.core.ownership;
import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.transformation;

import dagon.ext.newton;

// Имена dlib.math.transformation (up/forward/right — шаблоны) конфликтуют с
// базисом каркаса; нужные оси каркаса переименовываем локально.
import frame.frame : origin, frameForward = forward, frameRight = right;

import physics_world.physics;
import physics_world.terrain;

/// Heightfield-коллизия: ОДНА сплошная поверхность на всё окно из
/// (windowRadius·2+1)² тайлов. Внутренних швов нет — соседние тайлы сшиваются
/// в один буфер высот 1:1 (они и так аналитически бесшовны), поэтому колесо
/// не «зацепляется» на стыках отдельных тел. Буфер копируется в локальную
/// память: Newton 3.14 НЕ копирует высоты (хранит указатели), поэтому массив
/// живёт всё время жизни коллизии — освобождение в деструкторе.
final class TerrainWindowHeightfield : NewtonCollisionShape
{
    private float[] elevations_;
    private ubyte[] attributes_;

    /// `elevations` — высоты окна в порядке heightfield Newton по сетке
    /// (tilesPerSide·cells+1)², `cells` ячеек на тайл по обеим осям, `cell` —
    /// размер ячейки. Сетка сэмплирована на общих для соседних тайлов точках,
    /// поэтому граница между тайлами — одна и та же строка буфера.
    this(const float[] elevations, uint cells, uint tilesPerSide,
        float cell, NewtonPhysicsWorld world)
    {
        super(world);

        const uint spanCells = tilesPerSide * cells;
        const size_t n = (spanCells + 1) * (spanCells + 1);
        elevations_ = New!(float[])(n);
        attributes_ = New!(ubyte[])(n);
        elevations_[] = elevations[];
        attributes_[] = 0;

        newtonCollision = NewtonCreateHeightFieldCollision(world.newtonWorld,
            cast(int) spanCells + 1, cast(int) spanCells + 1, 1, // gridsDiagonals
            0, // elevationdatType: float
            elevations_.ptr, cast(char*) attributes_.ptr,
            1.0f, // verticalScale
            cell, cell, 0);
        NewtonCollisionSetUserData(newtonCollision, cast(void*) this);
    }

    ~this()
    {
        Delete(elevations_);
        Delete(attributes_);
    }
}

/**
 * Физическое окно земли вокруг машины.
 *
 * Держит в мире Newton сплошную поверхность из (windowRadius·2+1)² тайлов
 * вокруг фокуса (точка на плоскости каркаса forward×right), сшитую в один
 * heightfield — у выезда за окно тело пересобирается из общего shared-кэша
 * тайлов с новым центром. Каждому инстансу BuggyPhysics — собственный
 * TerrainWorld (у каждого заезда свой мир), кэш высот общий.
 */
final class TerrainWorld
{
    private NewtonPhysicsWorld world_;
    private TerrainSurface terrain_;
    private TerrainConfig cfg_;

    /// Полуширина окна в тайлах: окно (2·R+1)² тайлов, т.е. 5×5 при R = 2.
    private int windowRadius_ = 1;

    /// Единственное ground-тело окна и его центр (номер центрального тайла).
    private TerrainWindowHeightfield groundShape_;
    private NewtonRigidBody ground_;
    private int groundTx_ = int.max;
    private int groundTy_ = int.max;

    this(NewtonPhysicsWorld world, TerrainSurface terrain, const TerrainConfig cfg)
    {
        world_ = world;
        terrain_ = terrain;
        cfg_ = cfg;
    }

    /// Продвинуть окно за фокусом (точка на плоскости каркаса): ground окна
    /// пересобирается только при смене центрального тайла.
    void updateAround(const vec3 focus)
    {
        const TileIndex c = tileIndexAt(focus, cfg_);

        if (c.tx != groundTx_ || c.ty != groundTy_)
        {
            rebuildGround(c.tx, c.ty);
            groundTx_ = c.tx;
            groundTy_ = c.ty;
        }
    }

    /// Снести всё окно: обязательно вызвать до того, как мир уйдёт в пул
    /// (NewtonDestroyAllBodies) или будет уничтожен, — иначе двойное
    /// освобождение тел.
    void dispose()
    {
        if (ground_ !is null)
        {
            NewtonDestroyBody(ground_.newtonBody);
            world_.deleteOwnedObject(ground_);
            world_.deleteOwnedObject(groundShape_);
            ground_ = null;
            groundShape_ = null;
        }
    }

    /// Сборка единого ground-тела окна из cached-сеток девяти тайлов: нижний
    /// левый тайл (tx0, ty0) даёт первые `cells` ячеек по каждой оси, остальные
    /// — следующие блоки по `cells` ячеек. Смежные тайлы на общей грани имеют
    /// одинаковые высоты (аналитический кэш), поэтому сшивка бесшовна и без
    /// пересчёта шума.
    private void rebuildGround(int cx, int cy)
    {
        if (ground_ !is null)
        {
            NewtonDestroyBody(ground_.newtonBody);
            world_.deleteOwnedObject(ground_);
            world_.deleteOwnedObject(groundShape_);
            ground_ = null;
            groundShape_ = null;
        }

        const int R = windowRadius_;
        const int tx0 = cx - R, ty0 = cy - R;
        const uint tilesPerSide = cast(uint)(2 * R + 1);
        const uint cells = cfg_.cells;
        const uint spanCells = tilesPerSide * cells;
        const uint grid = spanCells + 1;
        const size_t n = grid * grid;

        auto elev = New!(float[])(n);
        foreach (di; 0 .. tilesPerSide)
            foreach (dj; 0 .. tilesPerSide)
            {
                immutable tile = terrain_.tileData(tx0 + cast(int) di,
                    ty0 + cast(int) dj);
                foreach (f; 0 .. cells + 1)
                    foreach (r; 0 .. cells + 1)
                    {
                        const size_t gi = di * cells + f;
                        const size_t gj = dj * cells + r;
                        elev[gi * grid + gj] = tile.heights[f * (cells + 1) + r];
                    }
            }

        const float cell = cfg_.tileSize / cast(float) cells;
        groundShape_ = New!TerrainWindowHeightfield(elev, cells, tilesPerSide,
            cell, world_);
        Delete(elev);

        ground_ = New!NewtonRigidBody(NewtonRigidBodyType.Static, groundShape_,
            0.0f, world_, world_);
        ground_.dynamic = false;
        const vec3 corner = origin
            + frameForward * (cast(float) tx0 * cfg_.tileSize - gridHalfShift(cfg_))
            + frameRight * (cast(float) ty0 * cfg_.tileSize - gridHalfShift(cfg_));
        ground_.setTransformation(translationMatrix(toNewtonPos(corner)));
        ground_.update(0.0);
    }
}