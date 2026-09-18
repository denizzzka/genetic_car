module physics_world.car;

import dlib.math.vector;
import frame.frame;
import physics_world.physics;

/// Полная машина: каркас багги вместе с якорями (колёсами).
class Buggy
{
    /// Каркас багги: узлы (позиции), балки, якоря.
    Frame frame;

    /// Смещение отображения, приводящее каркас к началу координат.
    /// Не меняет геометрию, применяется только при отрисовке.
    vec3 offset;

    /// Физическая всёленная машины, собранная `createPhysics()` из каркаса.
    CarPhysics physics;

    this(Frame frame, vec3 offset)
    {
        this.offset = offset;
        this.frame = frame;
    }

    /// Собрать физическую модель из каркаса (мир с землёй, тела балок
    /// и колёс). Стартовая позиция — в начале координат, низ самого низкого
    /// колеса на z == 0; display-офсет отображения не участвует. Повторный
    /// вызов безопасен: первая всёленная сохраняется.
    void createPhysics()
    {
        if (physics !is null)
            return;
        physics = new CarPhysics(frame, groundLift());
    }

    /// Вернуть всёленную: dispose и освободить ссылку. После этого можно
    /// снова `createPhysics()`.
    void disposePhysics()
    {
        if (physics !is null)
        {
            physics.dispose();
            physics = null;
        }
    }

    /// Один шаг симуляции; требует созданной физики.
    void step(double dt, float throttle)
    {
        if (physics !is null)
            physics.step(dt, throttle);
    }

    /// Подъём, ставящий низ самого низкого колеса на z == 0 (как в вьюере).
    private vec3 groundLift()
    {
        vec3 lift = vec3(0.0f);
        float minZ = float.max;
        foreach (a; frame.anchors)
        {
            const float z = frame.nodes[a.node].pos.z;
            if (z < minZ)
                minZ = z;
        }
        if (minZ < float.max)
            lift.z = wheelRadius - minZ;
        return lift;
    }
}