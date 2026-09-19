module physics_world.physics;

import std.math;
import std.exception : enforce;

import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;

import dagon.ext.newton;

/// Радиус колеса. Совпадает с внешним радиусом визуального тора и
/// коллизионного цилиндра: Solid-цилиндр Newton радиусом 0.3 × шириной 0.2.
enum float wheelRadius = 0.3f;

/// Колесо глубже этой отметки относительно земли — провал сквозь неё.
enum float physicsWheelBelow = -0.1f;

/// Колесо выше `wheelRadius` на эту величину — переворот или съезд с полосы.
enum float physicsWheelLift = 0.1f;

/// Внутренний радиус «отверстия» колеса (покрышка — полый цилиндр).
/// Нужен только для массы и тензора инерции; коллизия — по внешней
/// поверхности сплошного цилиндра, как и у исходной опорной функции.
enum float wheelInnerRadius = 0.22f;

/// Ширина колеса (длина оси цилиндра, равна внешней толщине тора).
enum float wheelWidth = 0.2f;

/// Плотность материала колеса, кг/м^3.
enum float wheelDensity = 400.0f;

/// Плотность материала балок каркаса, кг/м^3.
enum float beamDensity = 150.0f;

/// Допустимое заглубление балки под землю, м: контактная точка балки с
/// землёй ниже `-beamGroundEps` — это провал рамы и обрыв заезда. Мелкие
/// касания во время крена качения (доли сантиметра) прощаются, иначе
/// здоровая машина обнуляется на каждом бугорке.
enum float beamGroundEps = 0.02f;

/// Порог силы мотор-колёс, Н·м: ниже него привод заведомо не везёт машину,
/// и физический заезд запускать незачем.
enum float minMotorPower = 1.0f;

/// Наследственные коэффициенты трения и демпфирования: Newton считает их
/// по паре материалов / телу.
enum float groundFriction = 0.9f;
enum float bodyDamping = 0.5f;

/// Транспортное состояние тела: позиция и ориентация в координатах машины.
/// Совпадает с трансформацией Dagon-сущности под carRoot:
/// `entity.position = state.position; entity.rotation = state.orientation;`
struct BodyState
{
    Vector3f position;
    Quaternionf orientation;
}

/// Причина обрыва заезда из-за каркаса: балка или колесо задели внешний объект.
enum BeamFailure
{
    none,
    /// Балка каркаса касается земли (низко обвисшая рама).
    ground,
    /// Балка каркаса касается колеса (не через ступицу своего же якоря).
    wheel,
    /// Два колеса каркаса соприкасаются (в т.ч. когда их якоря в одной точке):
    /// тела теряют контакт с землёй, момент мотора глушится, машина стоит.
    wheelWheel,
}

/// Ньютоновская библиотека грузится один раз на процесс: bindbc resolve
/// символов динамической библиотеки не потокобезопасен, а физические заезды
/// идут на пуле воркеров. Гвард — ОТКРЫТЫЙ (не потоково-локальный) мьютекс.
private __gshared bool newtonLoaded_;
private __gshared Object newtonLoadLock_ = new Object();

/// Гарантировать загрузку libnewton.so перед созданием любого мира.
void ensureNewtonLoaded()
{
    if (newtonLoaded_)
        return;
    synchronized (newtonLoadLock_)
    {
        if (!newtonLoaded_)
        {
            auto sup = loadNewton();
            enforce(sup == NewtonSupport.newton314,
                "Не загрузилась libnewton.so. В каталоге сборки должны лежать "
                ~ "libnewton.so, libdgCore.so, libdgPhysics.so, libdgNewtonAvx.so "
                ~ "(копируются из dagon:newton); при ручном запуске добавь их "
                ~ "в LD_LIBRARY_PATH.");
            newtonLoaded_ = true;
        }
    }
}