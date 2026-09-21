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
 * Горизонт — плоскость XY, нормаль вверх — +Z.
 *
 * Заметка для графики: Dagon использует мир с Y вверх, поэтому при
 * отрисовке модель поворачивается (Z -> Y, см. viewer).
 */

/// Единичные векторы направлений развития в осях turtle-грамматики багги:
/// первая компонента дельты/позиции — «вперёд», вторая — «вправо», третья —
/// «вверх»; отрицания — backward/left/down. Константы используют стартовый
/// геном (initial_data) и тесты грамматики (buggygrammar). Это оси построения
/// каркаса, а не мировые оси автомобиля из блока «Система координат
/// автомобиля» выше (там right/forward названы иначе — см. physics).
immutable vec3 forward  = vec3( 1.0f,  0.0f,  0.0f);
immutable vec3 right    = vec3( 0.0f,  1.0f,  0.0f);
immutable vec3 up       = vec3( 0.0f,  0.0f,  1.0f);
immutable vec3 backward = vec3(-1.0f,  0.0f,  0.0f);
immutable vec3 left     = vec3( 0.0f, -1.0f,  0.0f);
immutable vec3 down     = vec3( 0.0f,  0.0f, -1.0f);

/// Тип якоря (колеса).
/// Колёса сделаны отдельными Anchor, а не общими "шарнирами", потому что
/// более общие шарниры не позволяют применять оптимизации в физическом
/// движке: специализированные контактные модели, расчёт трения качения,
/// крутящего момента и т.д. недоступны для универсальных шарниров.
enum AnchorKind { wheel, motorWheel }

/// Узел каркаса багги.
struct Node
{
    vec3 pos;
}

/// Якорь (колесо), прикреплённый к узлу по индексу.
struct Anchor
{
    size_t node;
    AnchorKind kind;
}

/// Тип балки каркаса.
enum BeamKind { normal }

/// Балка: пара индексов узлов в `Frame.nodes` и радиус трубы.
struct Beam
{
    size_t a, b;
    float radius = 0.04f;
    BeamKind kind = BeamKind.normal;
}

/// Каркас багги целиком: узлы, балки, якоря (колёса).
struct Frame
{
    Node[] nodes;
    Beam[] beams;
    Anchor[] anchors;

    /// Наследуемая сила мотор-колёс, Н·м (ген `motorPower`).
    float motorPower;

    /// Суммарная длина всех балок каркаса (приближение массы).
    @property float totalBeamLength() const
    {
        float len = 0.0f;
        foreach (b; beams)
            len += distance(nodes[b.a].pos, nodes[b.b].pos);
        return len;
    }
}

/// Стартовая сила мотор-колёс основателя, Н·м: около калиброванного оптимума,
/// чтобы простейшая машина сразу ехала; дальше эволюция подстроит её.
enum float initialMotorPower = 100.0f;

/**
 * Связность каркаса: все узлы достижимы из узла 0 по балкам.
 */
bool isConnected(const Frame frame)
{
    if (frame.nodes.length == 0)
        return true;

    bool[] visited = new bool[frame.nodes.length];
    size_t[] stack = [0];
    visited[0] = true;

    while (stack.length > 0)
    {
        const n = stack[$ - 1];
        stack.length -= 1;
        foreach (b; frame.beams)
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

unittest
{
    // Связный каркас: два колеса — переднее и ведущее заднее.
    Frame f;
    f.nodes = [
        Node(vec3(0.6f, 0.7f, 0.3f)),
        Node(vec3(0.6f, -0.7f, 0.3f)),
    ];
    f.beams = [Beam(0, 1, 0.05f)];
    f.anchors = [
        Anchor(0, AnchorKind.wheel),
        Anchor(1, AnchorKind.motorWheel),
    ];

    assert(f.nodes.length == 2);
    assert(f.beams.length == 1);
    assert(f.anchors.length == 2);
    assert(isConnected(f));
    assert(f.totalBeamLength > 0.0f);

    // Разорванный каркас не связан.
    Frame g;
    g.nodes = [
        Node(vec3(0.0f, 0.0f, 0.0f)),
        Node(vec3(0.0f, 1.0f, 0.0f)),
    ];
    assert(!isConnected(g));

    // Единственный узел связан тривиально.
    Frame h;
    h.nodes = [Node(vec3(0.0f, 0.0f, 0.5f))];
    assert(isConnected(h));
}