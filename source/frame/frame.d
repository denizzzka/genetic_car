module frame.frame;

import std.math;
import dlib.math.vector;

/*
 * Система координат автомобиля:
 *
 *   X — поперечная ось, направлена вправо (у автомобиля, стоящего носом вперёд).
 *   Y — продольная ось, направлена вперёд.
 *   Z — вертикальная ось, направлена вверх.
 *
 * Плоскость симметрии — X = 0 (содержит оси Y и Z). Правая половина
 * каркаса хранится с X >= 0. Горизонт — плоскость XY, нормаль вверх — +Z.
 *
 * Заметка для графики: Dagon использует мир с Y вверх, поэтому при
 * отрисовке модель поворачивается (Z -> Y, см. viewer).
 */
/// Полуось: узел считается лежащим на плоскости симметрии.
enum float planeEpsilon = 1e-4f;

/// Тип якоря (колеса).
/// Колёса сделаны отдельными Anchor, а не общими "шарнирами", потому что
/// более общие шарниры не позволяют применять оптимизации в физическом
/// движке: специализированные контактные модели, расчёт трения качения,
/// крутящего момента и т.д. недоступны для универсальных шарниров.
enum AnchorKind { wheel, motorWheel }

/// Узел правой половины каркаса. `pos.x >= 0`; `pos.x == 0` — узел на оси симметрии.
struct Node
{
    vec3 pos;
}

/// Якорь (колесо), прикреплённый к узлу по индексу.
/// Хранится на том же уровне иерархии, что и `Beam`.
struct Anchor
{
    size_t node;
    AnchorKind kind;
}

/// Тип балки.
enum BeamKind { normal, cross, axial }

/**
 * Балка: пара индексов узлов в `Frame.nodes`.
 *
 * `cross` — балка, строго перпендикулярная плоскости симметрии. Соединяет
 * правый узел `a` (`x > 0`) с осевым узлом `b` (`x == 0`), у которого те же
 * `y` и `z`, что у `a`. В полном каркасе она разворачивается в одну прямую
 * трубу `(x,y,z) -> (-x,y,z)` сквозь плоскость, без излома в осевом узле.
 *
 * `axial` — балка целиком лежит на оси симметрии (оба узла `x == 0`).
 */
struct Beam
{
    size_t a, b;
    float radius = 0.04f;
    BeamKind kind = BeamKind.normal;
}

/**
 * Половина каркаса (правая сторона).
 * Полный каркас строится зеркальным замыканием, см. `mirrorClosure`.
 */
struct Frame
{
    Node[] nodes;
    Beam[] beams;
    Anchor[] anchors;
}

/// Балка уже развёрнутого полного каркаса.
struct FullBeam
{
    size_t a, b;
    float radius;
    BeamKind kind = BeamKind.normal;
}

/// Полный двусторонний каркас, полученный из правой половины.
struct FullFrame
{
    /// Все узлы: правые и их зеркальные копии.
    vec3[] nodes;
    /// index половины -> индекс правого/осевого узла в `nodes`.
    size_t[] right;
    /// index половины -> индекс левого узла в `nodes` (== `right` для осевых).
    size_t[] left;
    FullBeam[] beams;
    /// Якоря (колёса), развёрнутые зеркальным замыканием.
    Anchor[] anchors;

    /// Суммарная длина всех балок полного каркаса (приближение массы).
    @property float totalBeamLength() const
    {
        float len = 0.0f;
        foreach (b; beams)
            len += distance(nodes[b.a], nodes[b.b]);
        return len;
    }
}

/// Зеркально отразить точку относительно плоскости симметрии x = 0.
vec3 mirrorX(const vec3 v)
{
    return vec3(-v.x, v.y, v.z);
}

/// Лежит ли точка на плоскости симметрии.
bool isOnPlane(const vec3 p)
{
    return fabs(p.x) <= planeEpsilon;
}

/**
 * Развёрнуть половину каркаса в полный.
 *
 * Узел с `x > 0` дублируется зеркально, узел на оси — остаётся один.
 * Обычная балка добавляется дважды (правая + зеркальная), кроме балок,
 * целиком лежащих на оси — они существуют в единственном экземпляре.
 *
 * Cross-балка — строгий перпендикуляр к плоскости: правый узел `a` (`x > 0`)
 * и осевой узел `b` (`x == 0`) на тех же `y, z`. В полном каркасе получается
 * одна прямая труба `(x,y,z) -> (-x,y,z)`, проходящая сквозь плоскость.
 *
 * Axial-балка целиком лежит на оси симметрии (оба узла с `x == 0`).
 *
 * Якоря (колёса) разворачиваются аналогично: якорь на осевом узле
 * остаётся один, на правом — дублируется зеркально.
 */
FullFrame mirrorClosure(const Frame frame)
{
    FullFrame full;
    full.right.length = frame.nodes.length;
    full.left.length = frame.nodes.length;

    foreach (i, node; frame.nodes)
    {
        if (isOnPlane(node.pos))
        {
            full.nodes ~= node.pos;
            full.right[i] = full.left[i] = full.nodes.length - 1;
        }
        else
        {
            full.nodes ~= node.pos;
            full.right[i] = full.nodes.length - 1;
            full.nodes ~= mirrorX(node.pos);
            full.left[i] = full.nodes.length - 1;
        }
    }

    foreach (beam; frame.beams)
    {
        if (beam.a == beam.b)
            continue;

        const aOffPlane = full.right[beam.a] != full.left[beam.a];
        const bOffPlane = full.right[beam.b] != full.left[beam.b];

        final switch (beam.kind)
        {
            case BeamKind.cross:
                assert(aOffPlane && !bOffPlane,
                    "cross-балка должна соединять правый узел с осевым");
                {
                    const pa = frame.nodes[beam.a].pos;
                    const pb = frame.nodes[beam.b].pos;
                    assert(isClose(pa.y, pb.y) && isClose(pa.z, pb.z),
                        "cross-балка должна быть перпендикулярна плоскости x == 0");
                    full.beams ~= FullBeam(full.right[beam.a], full.left[beam.a],
                        beam.radius, BeamKind.cross);
                }
                break;

            case BeamKind.axial:
                assert(!aOffPlane && !bOffPlane,
                    "осевая балка должна целиком лежать на x == 0");
                full.beams ~= FullBeam(full.right[beam.a], full.right[beam.b], beam.radius, BeamKind.axial);
                break;

            case BeamKind.normal:
                full.beams ~= FullBeam(full.right[beam.a], full.right[beam.b], beam.radius, BeamKind.normal);
                if (aOffPlane || bOffPlane)
                    full.beams ~= FullBeam(full.left[beam.a], full.left[beam.b], beam.radius, BeamKind.normal);
                break;
        }
    }

    foreach (anchor; frame.anchors)
    {
        const fullIdx = full.right[anchor.node];
        const mirrorIdx = full.left[anchor.node];

        full.anchors ~= Anchor(fullIdx, anchor.kind);
        if (fullIdx != mirrorIdx)
            full.anchors ~= Anchor(mirrorIdx, anchor.kind);
    }

    return full;
}

/**
 * Проверка связности полного каркаса: все узлы достижимы из узла 0 по балкам.
 *
 * Нужна именно после `mirrorClosure`, потому что связность правой половины
 * не гарантирует связности зеркального замыкания. Cross-балка в полном
 * каркасе соединяет правый узел со своим зеркалом, а не с осевым узлом
 * (см. `mirrorClosure`), поэтому узел, у которого одни лишь cross-балки,
 * превращается в отдельный «висящий» компонент. А без cross-балок и без
 * общего осевого узла зеркальные половины вообще не соединены между собой.
 */
bool isConnected(const FullFrame full)
{
    if (full.nodes.length == 0)
        return true;

    bool[] visited = new bool[full.nodes.length];
    size_t[] stack = [0];
    visited[0] = true;

    while (stack.length > 0)
    {
        const n = stack[$ - 1];
        stack.length -= 1;
        foreach (b; full.beams)
        {
            size_t next;
            if (b.a == n)
                next = b.b;
            else if (b.b == n)
                next = b.a;
            else
                continue;
            if (next < visited.length && !visited[next])
            {
                visited[next] = true;
                stack ~= next;
            }
        }
    }

    foreach (v; visited)
        if (!v)
            return false;
    return true;
}

/**
 * Проверка того, что для каждой балки полного каркаса в нём же есть
 * её зеркальное отражение. Балка на оси симметрии — собственное отражение.
 */
bool isSymmetric(const FullFrame full)
{
    foreach (b; full.beams)
    {
        if (b.a == b.b)
            continue;

        const vec3 ma = mirrorX(full.nodes[b.a]);
        const vec3 mc = mirrorX(full.nodes[b.b]);
        const float eps2 = planeEpsilon * planeEpsilon;

        bool found;
        foreach (bb; full.beams)
        {
            if (bb.a == bb.b)
                continue;
            const bool forward = distancesqr(full.nodes[bb.a], ma) <= eps2
                && distancesqr(full.nodes[bb.b], mc) <= eps2;
            const bool reverse = distancesqr(full.nodes[bb.a], mc) <= eps2
                && distancesqr(full.nodes[bb.b], ma) <= eps2;
            if (forward || reverse)
            {
                found = true;
                break;
            }
        }
        if (!found)
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------

unittest
{
    // Узлы половины: два колесных якоря справа, момент на оси, свободный узел.
    Frame frame;
    frame.nodes = [
        Node(vec3(0.6f,  1.0f, 0.3f)),  // переднее правое колесо
        Node(vec3(0.6f, -1.0f, 0.3f)),  // заднее правое колесо
        Node(vec3(0.0f,  0.0f, 0.5f)),  // мотор на оси
        Node(vec3(0.3f,  0.0f, 0.9f)),  // свободно, x > 0
    ];
    frame.anchors = [
        Anchor(0, AnchorKind.wheel),
        Anchor(1, AnchorKind.motorWheel),
    ];
    frame.beams = [
        Beam(0, 2),  // переднее колесо -> мотор (обычная, конец на оси)
        Beam(1, 2),  // заднее колесо -> мотор (обычная, конец на оси)
        Beam(2, 3),  // мотор -> свободный узел вправо
        Beam(3, 3),  // вырожденная, должна отбрасываться
    ];

    const full = mirrorClosure(frame);

    // 3 парных узла + 1 осевой = 3*2 + 1 = 7 узлов.
    assert(full.nodes.length == 7);
    assert(full.right.length == 4);
    assert(full.left.length == 4);

    // Осевой узел мотора не дублируется.
    assert(full.right[2] == full.left[2]);

    // Два якоря на правых узлах (x > 0) -> 4 якоря в полном каркасе.
    assert(full.anchors.length == 4);

    // Первый якорь (wheel на узле 0) даёт пару: right[0] и left[0].
    bool hasRightWheel, hasLeftWheel;
    foreach (a; full.anchors)
    {
        if (a.kind == AnchorKind.wheel && a.node == full.right[0])
            hasRightWheel = true;
        if (a.kind == AnchorKind.wheel && a.node == full.left[0])
            hasLeftWheel = true;
    }
    assert(hasRightWheel && hasLeftWheel, "wheel-якорь должен быть зеркально продублирован");

    // Второй якорь (motorWheel на узле 1) даёт пару: right[1] и left[1].
    bool hasRightMotor, hasLeftMotor;
    foreach (a; full.anchors)
    {
        if (a.kind == AnchorKind.motorWheel && a.node == full.right[1])
            hasRightMotor = true;
        if (a.kind == AnchorKind.motorWheel && a.node == full.left[1])
            hasLeftMotor = true;
    }
    assert(hasRightMotor && hasLeftMotor, "motorWheel-якорь должен быть зеркально продублирован");

    // Первая балка (0->2) разворачивается в пару обычных:
    // правый узел -> осевой и левый узел -> осевой.
    const mirroredBeam = FullBeam(full.left[0], full.left[2], frame.beams[0].radius);

    // Балка с концом на оси — не cross-.
    foreach (b; full.beams)
    {
        assert(b.kind != BeamKind.cross);
    }
    bool hasMirrored;
    foreach (b; full.beams)
    {
        if (b.a == mirroredBeam.a && b.b == mirroredBeam.b)
        {
            hasMirrored = true;
            break;
        }
    }
    assert(hasMirrored, "обычная балка не получила зеркальное продолжение");

    // Полный каркас симметричен.
    assert(isSymmetric(full));

    // Вырожденная балка (3,3) отброшена: 3 балки -> 3*2 = 6.
    assert(full.beams.length == 6);
}

unittest
{
    // Балка целиком на оси симметрии существует в одном экземпляре.
    Frame frame;
    frame.nodes = [
        Node(vec3(0.0f,  1.0f, 0.5f)),
        Node(vec3(0.0f, -1.0f, 0.5f)),
    ];
    frame.beams = [Beam(0, 1, 0.04f, BeamKind.axial)];

    const full = mirrorClosure(frame);

    // Оба узла на оси — не дублируются.
    assert(full.nodes.length == 2);
    // Балка одна.
    assert(full.beams.length == 1);
    assert(full.beams[0].kind == BeamKind.axial);
    assert(isSymmetric(full));
}

unittest
{
    // Cross-балка: правый узел a (x > 0) и осевой узел b (x == 0)
    // на тех же y,z. Разворачивается в одну прямую трубу (x,y,z) -> (-x,y,z).
    Frame frame;
    frame.nodes = [
        Node(vec3(0.6f, 1.0f, 0.3f)),
        Node(vec3(0.0f, 1.0f, 0.3f)),
    ];
    frame.beams = [
        Beam(0, 1, 0.04f, BeamKind.cross),
    ];

    const full = mirrorClosure(frame);

    // 1 парный узел + 1 осевой = 1*2 + 1 = 3 узла.
    assert(full.nodes.length == 3);
    assert(full.beams.length == 1);
    assert(full.beams[0].kind == BeamKind.cross);

    // Труба идёт от правого узла к его зеркалу.
    assert(full.beams[0].a == full.right[0]);
    assert(full.beams[0].b == full.left[0]);

    assert(isSymmetric(full));
}

unittest
{
    // Якорь на осевом узле не дублируется.
    Frame frame;
    frame.nodes = [
        Node(vec3(0.0f, 0.0f, 0.5f)),
    ];
    frame.anchors = [
        Anchor(0, AnchorKind.wheel),
    ];
    frame.beams = [];

    const full = mirrorClosure(frame);
    assert(full.anchors.length == 1);
    assert(full.anchors[0].node == full.right[0]);
    assert(full.anchors[0].kind == AnchorKind.wheel);
}

unittest
{
    // Пустой каркас — тривиально симметричен и корректен.
    const full = mirrorClosure(Frame.init);
    assert(full.nodes.length == 0);
    assert(full.beams.length == 0);
    assert(full.anchors.length == 0);
    assert(isSymmetric(full));
    assert(isConnected(full));
}

unittest
{
    // Связность полного каркаса: связность правой половины не гарантирует
    // связности зеркального замыкания.

    // Прецедент А: у off-plane узла 0 единственная балка — cross к осевому
    // узлу 1. Половина связана, но в полном каркасе cross-труба висит
    // отдельным компонентом (r0–l0), а осевой узел 1 не получает рёбер.
    {
        Frame f;
        f.nodes = [
            Node(vec3(0.3f, 0.0f, 0.0f)),
            Node(vec3(0.0f, 0.0f, 0.0f)),
        ];
        f.beams = [Beam(0, 1, 0.04f, BeamKind.cross)];
        assert(!isConnected(mirrorClosure(f)), "висящая cross-труба не должна быть связной");
    }

    // Прецедент Б: без cross-балок и общих осевых узлов зеркальные
    // половины соединены только внутри себя, но не друг с другом.
    {
        Frame f;
        f.nodes = [
            Node(vec3(0.3f, 0.0f, 0.0f)),
            Node(vec3(0.3f, 1.0f, 0.0f)),
        ];
        f.beams = [Beam(0, 1, 0.04f, BeamKind.normal)];
        assert(!isConnected(mirrorClosure(f)), "разорванные половины не должны быть связными");
    }

    // Осевой узел-мост: обе половины и весь каркас остаются связными.
    {
        Frame f;
        f.nodes = [
            Node(vec3(0.3f, 0.0f, 0.0f)),
            Node(vec3(0.0f, 0.0f, 0.0f)),
        ];
        f.beams = [Beam(0, 1, 0.04f, BeamKind.normal)];
        assert(isConnected(mirrorClosure(f)));
    }
}
