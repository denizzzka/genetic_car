module frame.frame;

import std.math;
import dlib.math.vector;
import physics_world.wheel : defaultWheelRadius;

/*
 * Система координат каркаса задаётся его собственным ортонормированным
 * базисом: right/forward/up и их отрицания (константы ниже). Им пользуются
 * стартовый геном (initial_data) и грамматика (buggygrammar).
 */

immutable vec3 right    = vec3( 1.0f,  0.0f,  0.0f);
immutable vec3 forward  = vec3( 0.0f, -1.0f,  0.0f);
immutable vec3 up       = vec3( 0.0f,  0.0f,  1.0f);
immutable vec3 backward = vec3( 0.0f,  1.0f,  0.0f);
immutable vec3 left     = vec3(-1.0f,  0.0f,  0.0f);
immutable vec3 down     = vec3( 0.0f,  0.0f, -1.0f);

/// Начало координат каркаса (нулевой вектор).
immutable vec3 origin = vec3(0.0f, 0.0f, 0.0f);

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

    /// Радиус колеса, м, свой у каждого якоря. По умолчанию —
    /// `defaultWheelRadius` из физического модуля колёс.
    float radius = defaultWheelRadius;
}

/// Тип обычной балки каркаса.
enum BeamKind { normal }

/// Эфемерная балка: связывает два узла (`Frame.nodes`), не несёт массы,
/// рендера и физического тела — только топология. Наследники добавляют
/// собственные свойства (радиус, `kind`).
class EphemeralBeam
{
    size_t a, b;

    this(size_t a, size_t b)
    {
        this.a = a;
        this.b = b;
    }
}

/// Обычная балка: эфемерная плюс радиус трубы и тип. Даёт каркасу массу,
/// рендер и физическое тело.
class Beam : EphemeralBeam
{
    float radius = 0.04f;
    BeamKind kind = BeamKind.normal;

    this(size_t a, size_t b, float radius = 0.04f, BeamKind kind = BeamKind.normal)
    {
        super(a, b);
        this.radius = radius;
        this.kind = kind;
    }
}

/// Каст к обычной балке: масса, рендер и физическое тело есть только у `Beam`.
Beam asBeam(const EphemeralBeam b)
{
    auto beam = cast(Beam) b;
    assert(beam !is null, "только обычная балка несёт радиус/массу/рендер");
    return beam;
}

/// Каркас багги целиком: узлы, балки, якоря (колёса).
struct Frame
{
    Node[] nodes;
    EphemeralBeam[] beams;
    Anchor[] anchors;

    /// Наследуемая сила мотор-колёс, Н·м (ген `motorPower`). Знак задаёт
    /// направление привода: отрицательный момент едет в обратную сторону.
    float motorPower;

    /// Суммарная длина всех обычных балок каркаса (приближение массы).
    /// Эфемерные балки массы не несут и в массу не входят.
    @property float totalBeamLength() const
    {
        float len = 0.0f;
        foreach (b; beams)
        {
            if (cast(Beam) b is null)
                continue;
            len += distance(nodes[b.a].pos, nodes[b.b].pos);
        }
        return len;
    }

    /// Направление «своей» балки узла — от самого узла к другому её концу.
    /// Каркас связный, поэтому балка у узла есть всегда.
    vec3 beamDirectionAt(size_t node) const
    {
        foreach (b; beams)
        {
            if (b.a == node)
                return nodes[b.b].pos - nodes[node].pos;
            if (b.b == node)
                return nodes[b.a].pos - nodes[node].pos;
        }
        assert(false, "у узла нет балки: каркас должен быть связным");
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
    f.beams = [new Beam(0, 1, 0.05f)];
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
        Node(origin),
        Node(right),
    ];
    assert(!isConnected(g));

    // Единственный узел связан тривиально.
    Frame h;
    h.nodes = [Node(vec3(0.0f, 0.0f, 0.5f))];
    assert(isConnected(h));

    // Эфемерная балка связывает узлы в топологии, но массы не несёт.
    Frame j;
    j.nodes = [Node(origin), Node(vec3(0.0f, 1.0f, 0.0f))];
    j.beams = [new EphemeralBeam(0, 1)];
    assert(isConnected(j));
    assert(j.totalBeamLength == 0.0f);
    assert(j.beamDirectionAt(0) == vec3(0.0f, 1.0f, 0.0f));
}