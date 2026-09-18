module physics_world.car;

import dlib.math.vector;
import frame.frame;

/// Полная машина: каркас багги вместе с якорями (колёсами).
class Buggy
{
    /// Каркас багги: узлы (позиции), балки, якоря.
    Frame frame;

    /// Смещение отображения, приводящее каркас к началу координат.
    /// Не меняет геометрию, применяется только при отрисовке.
    vec3 offset;

    this(Frame frame, vec3 offset)
    {
        this.offset = offset;
        this.frame = frame;
    }
}