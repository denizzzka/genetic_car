module physics_world.route;

import std.math;
import std.algorithm : min, max, clamp;

import dlib.math.vector;

import frame.frame : origin, frameForward = forward, frameRight = right;

/*
 * Маршрут живёт в базисе каркаса (см. frame.frame): линия на нулевой высоте
 * плоскости forward×right, без связи с рельефом. Курс маршрута всегда ведёт
 * в сторону forward: у каждого сегмента dot(dir, forward) > 0.
 *
 * Маршрут бесконечен: сегменты порождаются процедурно по детерминированной
 * последовательности, а хранимая полилиния — скользящее окно вокруг машины.
 * Память не растёт, финиша нет; счёт — средний темп продвижения по дуге.
 */

/// Полуширина коридора зачёта прогресса, м. Габарит каркаса требует запаса
/// (boxWidthHalf = 1.25). Вне коридора прогресс не начисляется.
enum float routeCorridorHalfWidth = 2.0f;

/// Шаг разбиения дуг на полилинию, м.
enum float routeStep = 1.0f;

/// Половина длины хранимого окна полилинии, м. Старые точки за этим окном
/// отбрасываются: проекция ищет ближайший сегмент лишь рядом с машиной.
enum float routeWindowHalfLength = 60.0f;

/// Предел отклонения курса от `forward`, рад. Строго меньше 90°, поэтому у
/// любого сегмента `dot(dir, forward) = cos(отклонение) > 0`: без него сумма
/// поворотов за цикл уводила бы маршрут назад.
enum float routeMaxDeviation = 1.2f;

/// Сегмент маршрута — дуга заданной длины и кривизны. Кривизна 0 — прямая,
/// знак — сторона поворота (положительная — вправо по курсу).
struct RouteSegment
{
    float length;
    float curvature;
}

/// Результат проекции: дуговая координата `s`, расстояние до маршрута и
/// индекс сегмента, на который точка спроецировалась.
struct RouteProjection
{
    float s;
    float distance;
    size_t segment;
}

/**
 * Бесконечный маршрут как скользящее окно полилинии.
 *
 * Участок хранится в абсолютной дуговой координате `s` от начала мира:
 * `arc[i]` — дуговая координата точки `points[i]`, а `baseS` — дуга
 * `points[0]`. Продвижение вперёд дописывает точки и отрезает хвост;
 * дуговая координата машины при этом не сбрасывается и не зависит от окна.
 */
struct Route
{
    vec3[] points;
    float[] arc;
    /// Дуга точки `points[0]`, м.
    float baseS;
    /// Индекс следующего сегмента, который будет сгенерирован.
    size_t nextSegment;
    /// Курс в конце полилинии: с него продолжается генерация.
    vec3 tailHeading;
    /// Индекс сегмента, ближайшего к прошлой проекции: поиск локализуется
    /// вокруг него, а не по всей полилинии.
    size_t cursor;

    /// Проекция точки на полилинию: дуговая координата ближайшей точки,
    /// перпендикулярное расстояние до неё и индекс её сегмента. Поиск — по
    /// сегментам рядом с прошлым курсором: дёшево и устойчиво к
    /// самопересечениям ленты.
    RouteProjection project(const vec3 p) const
    {
        RouteProjection best;
        best.distance = float.max;

        if (points.length < 2)
        {
            if (points.length == 1)
            {
                best.s = arc[0];
                best.distance = distance(p, points[0]);
            }
            return best;
        }

        const size_t last = points.length - 2;
        const size_t lo = (cursor > 1) ? cursor - 2 : 0;
        const size_t hi = min(cursor + 2, last);

        foreach (i; lo .. hi + 1)
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
                best.segment = i;
            }
        }
        return best;
    }
}

/// Детерминированная последовательность сегментов: чередование прямых,
/// плавных (R ≈ 15–25 м) и резких (R ≈ 3–5 м) поворотов. Строится по номеру
/// сегмента без хранения всего маршрута.
RouteSegment routeSegmentAt(size_t k)
{
    immutable float[9] lengths = [
        40.0f, 30.0f, 20.0f, 8.0f, 15.0f, 35.0f, 10.0f, 25.0f, 40.0f];
    immutable float[9] curvatures = [
        0.0f, 1.0f / 20.0f, 0.0f, 1.0f / 4.0f, 0.0f,
        -1.0f / 18.0f, 1.0f / 3.0f, -1.0f / 22.0f, 0.0f];
    return RouteSegment(lengths[k % 9], curvatures[k % 9]);
}

/// Начальное окно фиксированного маршрута. Геометрия детерминирована, поэтому
/// у каждого потребителя (особи в параллельном отборе, вьюера) своё окно:
/// общий мутабельный маршрут гонялся бы между потоками и сбивал бы курсор.
Route fixedRoute()
{
    Route r;
    r.baseS = 0.0f;
    r.nextSegment = 0;
    r.tailHeading = frameForward;
    r.points ~= origin;
    r.arc ~= 0.0f;
    ensureRouteAhead(r, routeWindowHalfLength);
    return r;
}

/// Держать курс в пределах ±`routeMaxDeviation` от `forward`. За пределом
/// поворот прекращается (курс идёт по границе), поэтому маршрут остаётся
/// направленным вперёд. Заодно нормирует курс.
private vec3 clampHeading(vec3 h)
{
    const float theta = atan2(h.x, dot(h, frameForward));
    const float t = clamp(theta, -routeMaxDeviation, routeMaxDeviation);
    return frameRight * sin(t) + frameForward * cos(t);
}

/// Дописать сегменты, пока конец полилинии не дойдёт до дуги `targetS`.
/// Зовётся при продвижении машины: цель — фокус плюс запас вперёд, поэтому
/// окно не растёт само по себе.
void ensureRouteAhead(ref Route route, float targetS)
{
    if (route.points.length < 1)
        return;

    while (route.arc[$ - 1] < targetS)
    {
        const seg = routeSegmentAt(route.nextSegment);

        const size_t steps = max(1, cast(size_t) ceil(seg.length / routeStep));
        const float ds = seg.length / steps;

        vec3 heading = route.tailHeading;
        vec3 pos = route.points[$ - 1];

        foreach (_; 0 .. steps)
        {
            const float dTheta = ds * seg.curvature;
            const float c = cos(dTheta);
            const float s = sin(dTheta);
            heading = vec3(
                heading.x * c - heading.y * s,
                heading.x * s + heading.y * c,
                0.0f);
            heading = clampHeading(heading);
            pos = pos + heading * ds;
            route.points ~= pos;
            route.arc ~= route.arc[$ - 1] + ds;
        }
        route.tailHeading = heading;
        route.nextSegment += 1;
    }
}

/// Отрезать точки, оставшиеся позади более чем на `keepBehind` метров от
/// дуги `s`, и сдвинуть курсор на новое начало окна. Память окна постоянна.
void trimRouteBehind(ref Route route, float s, float keepBehind)
{
    const float cutS = s - keepBehind;
    size_t drop;
    while (drop + 2 < route.arc.length && route.arc[drop + 1] < cutS)
        drop += 1;

    if (drop == 0)
        return;

    route.points = route.points[drop .. $];
    route.arc = route.arc[drop .. $];
    route.baseS = route.arc[0];
    route.cursor = (route.cursor > drop) ? route.cursor - drop : 0;
}

/// Мягкий штраф за съезд с коридора: 1 вне ширины, плавно падает к 0.
/// TODO: подобрать форму и жёсткость по поведению отбора.
float corridorFactor(float lateral)
{
    const float x = lateral / routeCorridorHalfWidth;
    return 1.0f / (1.0f + x * x);
}

/**
 * Прогресс машины по маршруту: дуговая координата фокуса, спроецированного
 * на фиксированный маршрут.
 *
 * Точка привязки — центр колесного footprint: устойчив к качке рамы, лежит
 * на линии контакта с землёй и дёшев (готовое среднее wheelStates, car.d).
 *
 * Прогресс не убывает: возвращается максимум из прежнего значения и новой
 * проекции, поэтому откат или разворот машины не отнимает заработанное.
 *
 * Продвижение тянет за собой окно полилинии: впереди дотягиваются сегменты,
 * позади отрезается хвост (память не растёт).
 */
double routeProgress(ref Route route, const vec3 focus, double previous)
{
    const proj = route.project(focus);
    double s = proj.s;

    const float lateral = proj.distance;
    // Вне коридора дуговая координата замирает: прогресс копится только
    // внутри коридора, штраф за съезд — множитель счёта, не обрыв.
    if (lateral > routeCorridorHalfWidth)
        s = previous;

    route.cursor = proj.segment;
    ensureRouteAhead(route, cast(float) max(s, previous) + routeWindowHalfLength);
    trimRouteBehind(route, cast(float) max(s, previous), routeWindowHalfLength);

    return max(s, previous);
}

version (unittest)
{
    /// Лента непрерывна: соседние точки стоят не дальше шага разбиения, а
    /// дуговые координаты строго растут.
    private bool continuous(const Route r)
    {
        foreach (i; 0 .. r.points.length - 1)
        {
            const float d = distance(r.points[i], r.points[i + 1]);
            if (d > routeStep + 1e-3f)
                return false;
            if (!(r.arc[i + 1] > r.arc[i]))
                return false;
        }
        return true;
    }
}

@("route: у каждого сегмента курс ведёт в сторону forward")
unittest
{
    Route r;
    r.baseS = 0.0f;
    r.tailHeading = frameForward;
    r.points ~= origin;
    r.arc ~= 0.0f;
    ensureRouteAhead(r, 400.0f);

    foreach (i; 0 .. r.points.length - 1)
    {
        const vec3 dir = r.points[i + 1] - r.points[i];
        assert(dot(dir, frameForward) > 0.0f);
    }
    assert(continuous(r));
}

@("route: есть и резкие, и плавные повороты")
unittest
{
    Route r;
    r.baseS = 0.0f;
    r.tailHeading = frameForward;
    r.points ~= origin;
    r.arc ~= 0.0f;
    ensureRouteAhead(r, 400.0f);

    float maxTurn = 0.0f;
    float minTurn = float.max;
    bool smooth;
    bool sharp;
    foreach (i; 1 .. r.points.length - 1)
    {
        const vec3 a = r.points[i] - r.points[i - 1];
        const vec3 b = r.points[i + 1] - r.points[i];
        const float turn = abs(atan2(cross(a, b).z, dot(a, b)));
        maxTurn = max(maxTurn, turn);
        minTurn = min(minTurn, turn);
        // Плавный поворот: R ≈ 18–22 м при шаге 1 м даёт ~0.045–0.055 рад.
        if (turn > 0.03f && turn < 0.12f)
            smooth = true;
        // Резкий поворот: R ≈ 3–4 м даёт ~0.25–0.33 рад.
        if (turn > 0.2f)
            sharp = true;
    }
    // Прямые участки дают нулевой излом, есть и плавные, и резкие повороты.
    assert(minTurn < 1e-4f);
    assert(smooth);
    assert(sharp);
    assert(maxTurn > 0.2f);
}

@("route: прогресс не убывает, окно не растёт")
unittest
{
    // Опора — длинная полилиния того же генератора: фокус идёт строго по
    // маршруту, как машина, а окно `r` продвигается за ним.
    Route refRoute;
    refRoute.baseS = 0.0f;
    refRoute.tailHeading = frameForward;
    refRoute.points ~= origin;
    refRoute.arc ~= 0.0f;
    ensureRouteAhead(refRoute, 400.0f);

    Route r;
    r.baseS = 0.0f;
    r.tailHeading = frameForward;
    r.points ~= origin;
    r.arc ~= 0.0f;
    ensureRouteAhead(r, routeWindowHalfLength);

    // Окно не длиннее двух половин маршрута плюс запас на перелёт целого
    // сегмента (самый длинный — 40 м) и округление шагов разбиения.
    enum size_t maxWindow =
        2 * cast(size_t)(routeWindowHalfLength / routeStep)
        + cast(size_t)(40.0f / routeStep) + 8;

    double prev = 0.0;
    foreach (k; 0 .. refRoute.points.length)
    {
        const double p = routeProgress(r, refRoute.points[k], prev);
        assert(p >= prev);
        prev = p;
        assert(r.points.length <= maxWindow);
    }
    assert(prev > refRoute.arc[$ - 1] - 1.0f);
}
