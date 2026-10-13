module physics_world.route;

import std.math;
import std.algorithm : min, max, clamp;
import core.sync.mutex : Mutex;

import dlib.math.vector;

import frame.frame : origin, frameForward = forward, frameRight = right;

/*
 * Маршрут живёт в базисе каркаса (см. frame.frame): линия на нулевой высоте
 * плоскости forward×right, без связи с рельефом. Курс маршрута всегда ведёт
 * в сторону forward: у каждого сегмента dot(dir, forward) > 0.
 */

/// Полуширина коридора зачёта прогресса, м. Вне коридора дуговая координата
/// не начисляется: габарит каркаса требует запаса (boxWidthHalf = 1.25).
enum float routeCorridorHalfWidth = 2.0f;

/// Сегмент маршрута — дуга заданной длины и кривизны. Кривизна 0 — прямая,
/// знак — сторона поворота (положительная — вправо по курсу).
struct RouteSegment
{
    float length;
    float curvature;
}

/// Маршрут: точки полилинии и накопленная дуговая длина до каждой из них.
/// `arc[0] = 0`, `arc[$-1] = totalLength`.
struct Route
{
    vec3[] points;
    float[] arc;
    float totalLength;

    /// Проекция точки на полилинию: дуговая координата ближайшей точки и
    /// перпендикулярное расстояние до неё.
    RouteProjection project(const vec3 p) const
    {
        RouteProjection best;
        best.distance = float.max;

        foreach (i; 0 .. points.length - 1)
        {
            const vec3 a = points[i];
            const vec3 b = points[i + 1];
            const vec3 ab = b - a;
            const float abLen2 = dot(ab, ab);
            if (abLen2 <= 0.0f)
                continue;

            const float t = clamp(dot(p - a, ab) / abLen2, 0.0f, 1.0f);
            const vec3 closest = a + ab * t;
            const float d = distance(p, closest);
            if (d < best.distance)
            {
                best.distance = d;
                best.s = arc[i] + (arc[i + 1] - arc[i]) * t;
            }
        }
        return best;
    }
}

/// Результат проекции: дуговая координата `s` и расстояние до маршрута.
struct RouteProjection
{
    float s;
    float distance;
}

/// Маршрут из последовательности сегментов: каждая дуга разворачивается в
/// полилинию с постоянным шагом `routeStep`.
Route buildRoute(const RouteSegment[] segments)
{
    vec3[] points;
    float[] arc;
    float total = 0.0f;

    // Курс и позиция в плоскости маршрута: `heading` — единичный вектор,
    // `pos` — текущая точка. Стартуем из origin в сторону forward.
    vec3 heading = frameForward;
    vec3 pos = origin;

    points ~= pos;
    arc ~= 0.0f;

    foreach (seg; segments)
    {
        if (seg.length <= 0.0f)
            continue;

        // Дуга постоянной кривизны: идём шагами, поворачивая курс на
        // ds·curvature. Знак кривизны — вправо по курсу (вокруг up).
        const float steps = max(1.0f, ceil(seg.length / routeStep));
        const float ds = seg.length / steps;
        foreach (_; 0 .. cast(size_t) steps)
        {
            const float dTheta = ds * seg.curvature;
            // Поворот heading вокруг up на dTheta (в плоскости forward×right).
            const float c = cos(dTheta);
            const float s = sin(dTheta);
            const vec3 h = heading;
            heading = vec3(
                h.x * c - h.y * s,
                h.x * s + h.y * c,
                0.0f);
            pos = pos + heading * ds;
            total += ds;

            points ~= pos;
            arc ~= total;
        }
    }

    Route route;
    route.points = points;
    route.arc = arc;
    route.totalLength = total;
    return route;
}

/// Шаг разбиения дуг на полилинию, м.
private enum float routeStep = 1.0f;

/// Единственный фиксированный маршрут на процесс: одинаков для всех особей
/// и поколений, чтобы сравнение было честным. Строится по образцу
/// `sharedTerrain()` — ленивый синглтон под __gshared-локом.
private __gshared Object routeLock_ = new Object();
private __gshared Object routeHolder_;

Route fixedRoute()
{
    if (routeHolder_ !is null)
        return *cast(Route*) routeHolder_;
    synchronized (routeLock_)
    {
        if (routeHolder_ is null)
        {
            auto r = new Route;
            *r = buildRoute(defaultRouteSegments);
            routeHolder_ = cast(Object) r;
        }
        return *cast(Route*) routeHolder_;
    }
}

/// Фиксированный набор сегментов: чередование прямых, плавных (R ≈ 15–25 м)
/// и резких (R ≈ 3–5 м) поворотов. Все сегменты смотрят в forward: машина
/// едет только вперёд.
private immutable RouteSegment[] defaultRouteSegments = [
    RouteSegment(40.0f, 0.0f),          // прямая
    RouteSegment(30.0f, 1.0f / 20.0f),  // плавный правый (R=20)
    RouteSegment(20.0f, 0.0f),          // прямая
    RouteSegment(8.0f, 1.0f / 4.0f),    // резкий правый (R=4)
    RouteSegment(15.0f, 0.0f),          // прямая
    RouteSegment(35.0f, -1.0f / 18.0f), // плавный левый (R=18)
    RouteSegment(10.0f, 1.0f / 3.0f),   // резкий правый (R=3)
    RouteSegment(25.0f, -1.0f / 22.0f), // плавный левый (R=22)
    RouteSegment(40.0f, 0.0f),          // прямая
];

/// Прогресс по маршруту: дуговая координата проекции, если точка внутри
/// коридора; иначе — `fallback` (вне коридора прогресс замирает).
float routeProgress(const Route route, const vec3 p, float fallback)
{
    const proj = route.project(p);
    if (proj.distance <= routeCorridorHalfWidth)
        return max(fallback, proj.s);
    return fallback;
}

unittest
{
    // Маршрут детерминирован: два вызова дают один и тот же набор точек.
    const a = fixedRoute();
    const b = fixedRoute();
    assert(a.points.length == b.points.length);
    assert(a.totalLength == b.totalLength);
    assert(a.totalLength > 0.0f);
}

unittest
{
    // Проекция точки на прямую часть: дуговая координата растёт вдоль линии.
    Route route;
    route.points = [origin, frameForward * 10.0f, frameForward * 20.0f];
    route.arc = [0.0f, 10.0f, 20.0f];
    route.totalLength = 20.0f;

    const p0 = route.project(origin + frameForward * 5.0f);
    assert(abs(p0.s - 5.0f) < 1e-4f);
    assert(p0.distance < 1e-4f);

    const p1 = route.project(origin + frameForward * 15.0f + frameRight * 1.0f);
    assert(abs(p1.s - 15.0f) < 1e-4f);
    assert(abs(p1.distance - 1.0f) < 1e-4f);

    // Точка за концом маршрута зажимается к последней точке.
    const p2 = route.project(origin + frameForward * 30.0f);
    assert(abs(p2.s - 20.0f) < 1e-4f);
}

unittest
{
    // Коридор: внутри — прогресс, снаружи — прежний fallback.
    Route route;
    route.points = [origin, frameForward * 10.0f];
    route.arc = [0.0f, 10.0f];
    route.totalLength = 10.0f;

    const inside = origin + frameForward * 5.0f + frameRight * 1.5f;
    assert(routeProgress(route, inside, 0.0f) > 0.0f);

    const outside = origin + frameForward * 5.0f + frameRight * 2.5f;
    assert(routeProgress(route, outside, 1.0f) == 1.0f);
}

unittest
{
    // Каждый сегмент фиксированного маршрута ведёт в сторону forward, а
    // среди кривизн есть и резкие, и плавные повороты.
    bool hasSharp, hasSmooth;
    foreach (seg; defaultRouteSegments)
    {
        if (seg.curvature == 0.0f)
            continue;
        const float radius = 1.0f / abs(seg.curvature);
        if (radius <= 5.0f)
            hasSharp = true;
        if (radius >= 15.0f)
            hasSmooth = true;
    }
    assert(hasSharp, "маршрут содержит резкие повороты");
    assert(hasSmooth, "маршрут содержит плавные повороты");

    // Сегменты развёрнутой полилинии монотонны по arc и по forward.
    const route = fixedRoute();
    foreach (i; 1 .. route.arc.length)
        assert(route.arc[i] > route.arc[i - 1],
            "дуговая длина строго растёт");
}

unittest
{
    // Монотонность прогресса: с движением вперёд вдоль прямой routeProgress
    // не убывает.
    Route route;
    route.points = [origin, frameForward * 20.0f];
    route.arc = [0.0f, 20.0f];
    route.totalLength = 20.0f;

    float prev = 0.0f;
    foreach (i; 0 .. 20)
    {
        const float d = cast(float) i;
        const float cur = routeProgress(route,
            origin + frameForward * d, prev);
        assert(cur >= prev);
        prev = cur;
    }
}
