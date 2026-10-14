module viewer.routevisualizer;

import std.math;
import std.algorithm : min;

import dlib.core.memory;
import dlib.math.vector;

import dagon;

import frame.frame : origin, frameForward = forward, frameUp = up;
import physics_world : dropHeight, TerrainSurface, BuggyPhysics, fixedRoute,
    Route, ensureRouteAhead, trimRouteBehind, routeWindowHalfLength;
import viewer.scene : carToScenePos;

/// Подъём ленты над рельефом, м. dagon не даёт polygonOffset из сущности,
/// поэтому depth bias делается подъёмом вершин: лента не тонет в поверхности
/// и не мерцает на её склонах.
enum float routeLift = 0.03f;

/// Полуширина рисуемой ленты, м: чуть уже коридора зачёта, чтобы край
/// коридора читался как запас, а не как обрыв прогресса.
enum float routeVisualHalfWidth = 1.5f;

/// Ёмкость меша ленты: пара вершин на точку и по два треугольника на сегмент.
/// Окно маршрута держится в `2·routeWindowHalfLength` плюс перелёт сегмента,
/// поэтому этой ёмкости хватает с запасом.
enum size_t routeMeshCap = 512;

/**
 * Второй слой поверх поверхности: лента вдоль бесконечного маршрута.
 *
 * Вершины ленты лежат на высоте рельефа плюс `routeLift` и переводятся в
 * координаты сцены через `carToScenePos` — как тайлы рельефа, сущность
 * висит в корне сцены. Меш один и переиспользуется: при сдвиге скользящего
 * окна маршрута вершины переписываются на месте, без New!/Delete в кадре.
 */
final class RouteVisualizer
{
    private Scene scene_;
    private TerrainSurface terrain_;
    private Material material_;
    private Mesh mesh_;
    private Entity entity_;
    /// Скользящее окно маршрута вьюера: своё, независимое от фитнеса.
    private Route route_;

    this(Scene scene, TerrainSurface terrain)
    {
        scene_ = scene;
        terrain_ = terrain;

        mesh_ = New!Mesh(scene.assetManager);
        mesh_.vertices = New!(Vector3f[])(routeMeshCap);
        mesh_.normals = New!(Vector3f[])(routeMeshCap);
        mesh_.texcoords = New!(Vector2f[])(routeMeshCap);
        mesh_.indices = New!(uint[3][])(routeMeshCap);

        material_ = scene.addMaterial();
        material_.baseColorFactor = Color4f(0.85f, 0.85f, 0.85f, 0.2f);
        material_.emissionFactor = Color4f(0.12f, 0.12f, 0.12f, 0.2f);
        material_.roughnessFactor = 0.6f;
        material_.metallicFactor = 0.0f;
        // Непрозрачный материал идёт в отложенный проход, где Geometry.frag
        // отсекает фрагменты по alphaTestThreshold (0.5), и alpha 0.3 стирает
        // ленту целиком. Transparent уводит её в forward-проход с блендингом.
        material_.blendMode = Transparent;

        entity_ = scene_.addEntity();
        entity_.drawable = mesh_;
        entity_.material = material_;

        route_ = fixedRoute();
    }

    /// Передвинуть ленту за живой машиной.
    void update(BuggyPhysics live)
    {
        const vec3 focus = (live is null) ? origin : live.surfaceFocus;
        rebuild(focus);
    }

    /// Передвинуть ленту за точкой фокуса (виртуальная багги: без физики).
    void updateFocus(const vec3 focus)
    {
        rebuild(focus);
    }

    /// Вернуть окно к старту маршрута: живой заезд начинается с начала.
    void reset()
    {
        route_ = fixedRoute();
        rebuild(origin);
    }

    /// Продвинуть окно маршрута к фокусу и переписать ленту по нему.
    private void rebuild(const vec3 focus)
    {
        const proj = route_.project(focus);
        route_.cursor = proj.segment;
        ensureRouteAhead(route_, proj.s + routeWindowHalfLength);
        trimRouteBehind(route_, proj.s, routeWindowHalfLength);
        rebuild(route_);
    }

    /// Перестроить ленту по текущему окну маршрута.
    private void rebuild(const Route route)
    {
        const auto pts = route.points;
        // Меш фиксированной ёмкости: пара вершин на точку и по два треугольника
        // на сегмент. Лишние точки за ёмкостью отбрасываются, а не переполняют.
        const size_t n = min(pts.length, routeMeshCap / 2);
        if (n < 2)
            return;

        const size_t quads = n - 1;

        size_t vi;
        size_t ii;
        const float baseS = route.arc[0];
        foreach (k; 0 .. n)
        {
            const vec3 p = pts[k];
            // Касательная по соседям: крайние точки берут одностороннюю.
            const vec3 a = pts[k > 0 ? k - 1 : 0];
            const vec3 b = pts[k + 1 < n ? k + 1 : n - 1];
            vec3 tan = b - a;
            tan.z = 0.0f;
            const float len = tan.length;
            const vec3 dir = (len > 1e-6f) ? tan / len : frameForward;
            const vec3 side = cross(vec3(0.0f, 0.0f, 1.0f), dir).normalized;

            const vec3 base = p
                + frameUp * (terrain_.heightAt(p) + dropHeight + routeLift);
            const vec3 l = base - side * routeVisualHalfWidth;
            const vec3 r = base + side * routeVisualHalfWidth;

            mesh_.vertices[vi + 0] = carToScenePos(l);
            mesh_.vertices[vi + 1] = carToScenePos(r);
            mesh_.normals[vi + 0] = carToScenePos(frameUp);
            mesh_.normals[vi + 1] = carToScenePos(frameUp);
            const float v = route.arc[k] - baseS;
            mesh_.texcoords[vi + 0] = Vector2f(0.0f, v);
            mesh_.texcoords[vi + 1] = Vector2f(1.0f, v);

            vi += 2;
        }

        foreach (k; 0 .. quads)
        {
            const uint i0 = cast(uint)(k * 2);
            const uint i1 = i0 + 1;
            const uint i2 = i0 + 2;
            const uint i3 = i0 + 3;
            mesh_.indices[ii++] = [i0, i2, i1];
            mesh_.indices[ii++] = [i1, i2, i3];
        }

        mesh_.dataReady = true;
        mesh_.calcBoundingBox();
        mesh_.prepareVAO();
    }
}
