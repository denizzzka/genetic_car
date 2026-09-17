module frame.buggy;

import std.algorithm : filter;
import std.range : walkLength;
import dlib.math.vector;
import frame.frame;

HalfFrame buggyFrame()
{
    HalfFrame f;

    size_t node(vec3 pos)
    {
        f.nodes ~= Node(pos);
        return f.nodes.length - 1;
    }

    void beam(size_t a, size_t b, float radius = 0.04f, BeamKind kind = BeamKind.normal)
    {
        f.beams ~= Beam(a, b, radius, kind);
    }

    void anchor(size_t nodeIdx, AnchorKind kind)
    {
        f.anchors ~= Anchor(nodeIdx, kind);
    }

    // Вершина — небольшая поперечная перекладина (cross-кап): осевой узел 0
    // (seed на оси симметрии) и правый конец перекладины; левый конец
    // появляется при зеркальном замыкании, и весь кап ложится одной прямой
    // поперёк машины с узлом в середине. Рёбра пирамиды — раскосы от центра
    // перекладины к колёсам.
    const apexCenter = node(vec3(0.00f, 0.00f, 1.05f));
    const apexRight = node(vec3(0.26f, 0.00f, 1.05f));

    const wheelFrontRight = node(vec3(0.60f, 0.70f, 0.10f));
    const wheelRearRight = node(vec3(0.60f, -0.70f, 0.10f));

    anchor(wheelFrontRight, AnchorKind.wheel);
    anchor(wheelRearRight, AnchorKind.motorWheel);

    const strutRadius = 0.05f;
    const crossRadius = 0.03f;

    // Поперечная перекладина вершины (вторая половина появляется
    // при зеркальном замыкании).
    beam(apexCenter, apexRight, crossRadius);

    // Рёбра пирамиды от центра перекладины к колёсам (после зеркального
    // замыкания — все четыре).
    beam(apexCenter, wheelFrontRight, strutRadius);
    beam(apexCenter, wheelRearRight, strutRadius);

    return f;
}

size_t anchorCount(const HalfFrame f)
{
    return f.anchors.length;
}

size_t planeNodeCount(const HalfFrame f)
{
    return f.nodes.filter!(n => isOnPlane(n.pos)).walkLength();
}

unittest
{
    const f = buggyFrame();

    // Вершина — небольшая поперечная перекладина: осевой центр + правый
    // конец + 2 правых колеса. Балки: 2 половинки перекладины + 2 раскоса.
    assert(f.nodes.length == 4);
    assert(f.beams.length == 3);
    assert(f.anchors.length == 2);

    const full = mirrorClosure(f);
    assert(isSymmetric(full));
    assert(isConnected(full));
    assert(full.nodes.length == 2 * f.nodes.length - planeNodeCount(f));
    // Полный каркас: 2 половинки перекладины + 4 раскоса = 6 рёбер,
    // 4 колеса.
    assert(full.beams.length == 6);
    assert(full.anchors.length == 4);
    assert(full.totalBeamLength > 0.0f);
    // Задние колёса ведущие.
    foreach (a; f.anchors)
        assert(a.kind == AnchorKind.motorWheel || a.kind == AnchorKind.wheel);
}
