module physics_world.terrainworld;

import std.math;
import std.algorithm : max, min;

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

/// Heightfield-коллизия одного тайла земли. Буферы копируются из immutable-
/// тайла 1:1 (без переворота строк): индексу (kx вдоль forward, ky вдоль
/// right) отвечает буфер heights[ky·W + kx], а Newton читает с угла (0,0)
/// вдоль локальных +X и +Z.
/// Перенос тайла на место — трансформацией тела (toNewtonPos угла тайла), а
/// не shape: матрица на heightfield внутри shape даёт NaN AABB
/// (см. GroundHeightfield).
final class TerrainHeightfield : NewtonCollisionShape
{
    // Newton 3.14 НЕ копирует высоты (хранит указатели), поэтому буферы живут
    // всё время жизни коллизии — освобождение в деструкторе.
    private float[] elevations_;
    private ubyte[] attributes_;

    this(immutable TerrainTileData tile, const TerrainConfig cfg,
        NewtonPhysicsWorld world)
    {
        super(world);

        const uint W = cfg.cells + 1;
        const size_t n = W * W;
        elevations_ = New!(float[])(n);
        attributes_ = New!(ubyte[])(n);
        foreach (i; 0 .. n)
        {
            elevations_[i] = tile.heights[i];
            attributes_[i] = 0;
        }

        const float cell = cfg.tileSize / cast(float) cfg.cells;
        newtonCollision = NewtonCreateHeightFieldCollision(world.newtonWorld,
            cast(int) W, cast(int) W, 1, // gridsDiagonals
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

/// Плотность материала булыжника, кг/м³ (скальная порода).
enum float boulderDensity = 2400.0f;

/// Динамический булыжник: сфера, которую можно сдвинуть колесом.
/// Обычный NewtonRigidBody (НЕ NewtonCarBody): контакт с балкой-сенсором
/// просто регистрируется и не фейлит заезд, а колёса толкают его честно.
final class TerrainBoulderBody : NewtonRigidBody
{
    this(float radius, float mass, NewtonPhysicsWorld world)
    {
        super(NewtonRigidBodyType.Dynamic, New!NewtonSphereShape(radius, world),
            mass, world, world);
    }
}

/// Живой булыжник для вьюера: кем-ключ тайла, индекс в нём и текущая поза.
struct LiveBoulder
{
    long key;
    size_t index;
    Vector3f position; ///< координаты мира dagon (= Newton)
    float radius;
}

/// Один стримингуемый тайл поверхности: ground-тело с heightfield и статично
/// наложенные динамические булыжники.
private struct TileEntry
{
    TerrainHeightfield shape;
    NewtonRigidBody ground;
    NewtonRigidBody[] boulders;
}

/**
 * Физическое окно земли вокруг машины.
 *
 * Держит в мире Newton кольцо из (windowRadius·2+1)² тайлов вокруг фокуса
 * (точка на плоскости каркаса forward×right): по мере езды вперёд снесённые
 * тайлы уничтожаются, новые — создаются из общего shared-кэша. Каждому
 * инстансу BuggyPhysics — собственный TerrainWorld (у каждого заезда свой
 * мир), кэш высот общий.
 */
final class TerrainWorld
{
    private NewtonPhysicsWorld world_;
    private TerrainSurface terrain_;
    private TerrainConfig cfg_;
    private TileEntry[long] tiles_;

    /// Полуширина окна в тайлах: окно (2·R+1)² тайлов, т.е. 5×5 при R = 2.
    private int windowRadius_ = 1;

    this(NewtonPhysicsWorld world, TerrainSurface terrain, const TerrainConfig cfg)
    {
        world_ = world;
        terrain_ = terrain;
        cfg_ = cfg;
    }

    /// Продвинуть окно за фокусом (точка на плоскости каркаса): подвезти
    /// недостающие тайлы, снести выпавшие за окно.
    void updateAround(const vec3 focus)
    {
        const float S = cfg_.tileSize;
        const int cx = cast(int) floor(focus.x / S);
        const int cy = cast(int) floor(focus.y / S);
        const int R = windowRadius_;

        const int tx0 = cx - R, tx1 = cx + R;
        const int ty0 = cy - R, ty1 = cy + R;
        foreach (tx; tx0 .. tx1 + 1)
            foreach (ty; ty0 .. ty1 + 1)
                ensureTile(tx, ty);

        long[] stale;
        foreach (key, ref e; tiles_)
        {
            const int tx = tileX(key), ty = tileY(key);
            if (tx < tx0 || tx > tx1 || ty < ty0 || ty > ty1)
                stale ~= key;
        }
        foreach (key; stale)
            removeTile(key);
    }

    /// Свежие булыжники окна (для вьюера при живом заезде).
    LiveBoulder[] activeBoulders()
    {
        LiveBoulder[] res;
        foreach (key, e; tiles_)
        {
            size_t i = 0;
            foreach (b; e.boulders)
            {
                b.update(0.0);
                LiveBoulder lb;
                lb.key = key;
                lb.index = i;
                lb.position = b.position.xyz;
                lb.radius = boulderRadius(b);
                res ~= lb;
                i++;
            }
        }
        return res;
    }

    /// Снести все тайлы окна: обязательно вызвать до того, как мир уйдёт в
    /// пул (NewtonDestroyAllBodies) или будет уничтожен, — иначе двойное
    /// освобождение тел.
    void dispose()
    {
        foreach (key; tiles_.keys)
            removeTile(key);
        tiles_ = null;
    }

    private void ensureTile(int tx, int ty)
    {
        const long key = tileKey(tx, ty);
        if (key in tiles_)
            return;

        immutable tile = terrain_.tileData(tx, ty);
        const vec3 corner = origin
            + frameForward * (cast(float) tx * cfg_.tileSize)
            + frameRight * (cast(float) ty * cfg_.tileSize);

        TileEntry e;
        e.shape = New!TerrainHeightfield(tile, cfg_, world_);
        e.ground = New!NewtonRigidBody(NewtonRigidBodyType.Static, e.shape,
            0.0f, world_, world_);
        e.ground.dynamic = false;
        e.ground.setTransformation(translationMatrix(toNewtonPos(corner)));
        e.ground.update(0.0);

        foreach (bi, bd; tile.boulders)
        {
            const vec3 ball = corner + frameForward * bd.alongForward
                + frameRight * bd.alongRight;
            const float r = bd.radius;
            const float h = terrain_.heightAt(ball);
            const float m = 4.0f / 3.0f * PI * r * r * r * boulderDensity;
            const float i = 0.4f * m * r * r;

            auto b = New!TerrainBoulderBody(r, m, world_);
            b.dynamic = true;
            b.gravity = gravity;
            b.setMassMatrix(m, i, i, i);
            // Поднятие along `up` над поверхностью, а затем в мир Newton.
            const vec3 center = toNewtonPos(ball)
                + vec3(0.0f, h + r, 0.0f);
            b.setTransformation(translationMatrix(center));
            b.update(0.0);
            e.boulders ~= b;
        }

        tiles_[key] = e;
    }

    private void removeTile(long key)
    {
        auto e = tiles_[key];
        foreach (b; e.boulders)
        {
            NewtonDestroyBody(b.newtonBody);
            world_.deleteOwnedObject(b);
        }
        NewtonDestroyBody(e.ground.newtonBody);
        world_.deleteOwnedObject(e.ground);
        world_.deleteOwnedObject(e.shape);
        tiles_.remove(key);
    }

    private static float boulderRadius(const NewtonRigidBody b)
    {
        return (cast(NewtonSphereShape) b.collisionShape).radius;
    }
}