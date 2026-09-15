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

/**
 * Что крепится в точке каркаса.
 * `none` — свободный (эволюционируемый) структурный узел.
 */
enum AnchorKind { none, wheel, motor, shock, spring, axle }

/// Узел правой половины каркаса. `pos.x >= 0`; `pos.x == 0` — узел на оси симметрии.
struct Node
{
    vec3 pos;
    AnchorKind kind = AnchorKind.none;
}

/// Тип балки.
enum BeamKind { normal, cross, axial }

/**
 * Балка: пара индексов узлов в `Frame.nodes`.
 *
 * `cross` — балка, заданная ДВУМЯ узлами правой половины (оба `x > 0`):
 * в полном каркасе одна труба соединяет правый узел `a` с зеркалом левого
 * узла `b`, а вторая — зеркало правого `a` с правым `b`. Такая труба прямая
 * и проходит сквозь плоскость симметрии, не преломляясь в осевом узле.
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
 * Cross-балка задаётся двумя узлами правой половины (оба `x > 0`):
 * она проходит прямым лучом сквозь плоскость симметрии, соединяя узел `a`
 * с зеркалом узла `b` (и наоборот). Балка, у которой конец лежит на оси
 * (`x == 0`), считается обычной, а не cross-.
 *
 * Axial-балка целиком лежит на оси симметрии (оба узла с `x == 0`).
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
                assert(aOffPlane && bOffPlane,
                    "cross-балка должна соединять два узла с x > 0");
                full.beams ~= FullBeam(full.right[beam.a], full.left[beam.b], beam.radius, BeamKind.cross);
                full.beams ~= FullBeam(full.left[beam.a], full.right[beam.b], beam.radius, BeamKind.cross);
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

    return full;
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
            if (distancesqr(full.nodes[bb.a], ma) <= eps2
                && distancesqr(full.nodes[bb.b], mc) <= eps2)
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
    import std.math : approxEqual;

    // Узлы половины: два колесных якоря справа, момент на оси, свободный узел.
    Frame frame;
    frame.nodes = [
        Node(vec3(0.6f,  1.0f, 0.3f), AnchorKind.wheel),  // переднее правое колесо
        Node(vec3(0.6f, -1.0f, 0.3f), AnchorKind.wheel),  // заднее правое колесо
        Node(vec3(0.0f,  0.0f, 0.5f), AnchorKind.motor),  // мотор на оси
        Node(vec3(0.3f,  0.0f, 0.9f), AnchorKind.spring), // свободно, x > 0
    ];
    frame.beams = [
        Beam(0, 2),  // правое колесо -> мотор (обычная, конец на оси)
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

    // Вырожденная балка (3,3) отброшена: 3 балки -> 3*2 - 1 = 5...
    // На самом деле: 0->2, 1->2, 2->3 дают по 2 балки (6), (3,3) отброшена.
    assert(full.beams.length == 6);
}

unittest
{
    // Балка целиком на оси симметрии существует в одном экземпляре.
    Frame frame;
    frame.nodes = [
        Node(vec3(0.0f,  1.0f, 0.5f), AnchorKind.axle),
        Node(vec3(0.0f, -1.0f, 0.5f), AnchorKind.spring),
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
    // Cross-балка: оба узла с x > 0. Разворачивается в пару прямых труб
    // сквозь плоскость: правый a <-> левый b и левый a <-> правый b.
    Frame frame;
    frame.nodes = [
        Node(vec3(0.6f,  1.0f, 0.3f)),
        Node(vec3(0.6f, -1.0f, 0.3f)),
    ];
    frame.beams = [
        Beam(0, 1, 0.04f, BeamKind.cross),
    ];

    const full = mirrorClosure(frame);

    assert(full.nodes.length == 4);
    assert(full.beams.length == 2);
    foreach (b; full.beams)
        assert(b.kind == BeamKind.cross);

    // Обе трубы симметричного каркаса: a->зеркало(b) и зеркало(a)->b.
    const прямой = FullBeam(full.right[0], full.left[1], 0.04f, BeamKind.cross);
    const зеркальный = FullBeam(full.left[0], full.right[1], 0.04f, BeamKind.cross);
    bool hasStraight, hasMirrored;
    foreach (b; full.beams)
    {
        if (b.a == прямой.a && b.b == прямой.b)
            hasStraight = true;
        if (b.a == зеркальный.a && b.b == зеркальный.b)
            hasMirrored = true;
    }
    assert(hasStraight, "нет прямой трубы a -> зеркало(b)");
    assert(hasMirrored, "нет зеркальной трубы зеркало(a) -> b");

    assert(isSymmetric(full));
}

unittest
{
    // Пустой каркас — тривиально симметричен и корректен.
    const full = mirrorClosure(Frame.init);
    assert(full.nodes.length == 0);
    assert(full.beams.length == 0);
    assert(isSymmetric(full));
}

