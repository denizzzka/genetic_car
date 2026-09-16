module car.car;

import dlib.math.vector;
import frame.frame;

/// Полная машина. Строится из правой половины каркаса зеркальным
/// замыканием и хранит уже развёрнутую двустороннюю
/// геометрию вместе с якорями. Позже сюда добавим физику.
class Buggy
{
    /// Полный каркас: узлы (позиции), балки, соответствие правых/осевых.
    FullFrame full;

    /// Якоря полного каркаса, выровнены с `full.nodes`.
    AnchorKind[] kinds;

    /// Смещение отображения, приводящее каркас к началу координат.
    /// Не меняет геометрию, применяется только при отрисовке.
    vec3 offset;

    this(const Frame half, vec3 offset = vec3(0.0f))
    {
        this.offset = offset;
        full = mirrorClosure(half);

        kinds.length = full.nodes.length;
        foreach (i, node; half.nodes)
        {
            kinds[full.right[i]] = node.kind;
            kinds[full.left[i]] = node.kind;
        }
    }
}

