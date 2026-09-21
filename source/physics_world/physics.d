module physics_world.physics;

import std.math;
import std.algorithm : min;
import std.exception : enforce;

import dlib.core.memory;
import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;
import dlib.math.transformation;

import dagon.ext.newton;

import frame.frame : Frame, Node, origin;

/// Радиус колеса. Совпадает с внешним радиусом визуального тора и
/// коллизионного цилиндра: Solid-цилиндр Newton радиусом 0.3 × шириной 0.2.
enum float wheelRadius = 0.3f;

/// Колесо глубже этой отметки относительно земли — провал сквозь неё.
enum float physicsWheelBelow = -0.1f;

/// Колесо выше `wheelRadius` на эту величину — переворот или съезд с полосы.
enum float physicsWheelLift = 0.1f;

/// Застой по курсу: машина, которая столько секунд симуляции не набирает
/// нового продвижения вперёд по курсу (-Y каркаса), сходит с дистанции.
enum double stallSeconds = 30.0;

/// Чистый набег вперёд по курсу, обнуляющий таймер застоя: дрожание и
/// качание на месте (< порога) продвижением не считаются.
enum float stallProgressEps = 0.1f;

/// Раскладка каркаса «на старт»: центрирует горизонтально (средняя X/Y узлов —
/// в ноль) и сажает низом самого низкого колеса на землю (min Z якоря →
/// wheelRadius). Единый способ поставить машину — им пользуются и грамматика
/// (`develop`), и физика (`BuggyPhysics`), и витрина.
vec3 placeOffset(const Frame f)
{
    vec3 c = origin;
    foreach (n; f.nodes)
        c += n.pos;
    if (f.nodes.length > 0)
        c /= f.nodes.length;

    float minZ = float.max;
    foreach (a; f.anchors)
        minZ = min(minZ, f.nodes[a.node].pos.z);
    const float dz = (minZ < float.max) ? wheelRadius - minZ : 0.0f;

    return vec3(-c.x, -c.y, dz);
}

/// Копия каркаса, разложенная `placeOffset`: узлы сдвинуты так, что каркас
/// отцентрован и низом колеса стоит на земле. Массивы копируются — узел
/// никогда не делит память с исходным каркасом.
Frame placedFrame(const Frame f)
{
    const vec3 off = placeOffset(f);
    Frame r;
    r.nodes = f.nodes.dup;
    r.beams = f.beams.dup;
    r.anchors = f.anchors.dup;
    r.motorPower = f.motorPower;
    foreach (ref n; r.nodes)
        n.pos += off;
    return r;
}

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

/// Ориентация физического мира: Newton держит «вверх» вдоль своей Y (земля —
/// плоскость XZ, у heightfield'а ось высоты — Y), а у каркаса и вьюера
/// «вверх» — Z. Мир Newton — это геометрия каркаса, повёрнутая вокруг X на
/// −90°: (x, y, z)каркас → (x, z, −y)newton. Все положения и ориентации тел,
/// уходящие в Newton и возвращающиеся из него, проходят через переводы ниже.
/// Значение из rotationQuaternion(Vector3f(1,0,0), −π/2) записано литералом:
/// сама функция не умеет CTFE.
immutable Quaternionf carToNewtonQuat =
    Quaternionf(-0.70710678f, 0.0f, 0.0f, 0.70710678f);

/// Точка из координат каркаса в координаты мира Newton.
vec3 toNewtonPos(const vec3 carPos)
{
    return vec3(carPos.x, carPos.z, -carPos.y);
}

/// Точка из координат мира Newton в координаты каркаса.
vec3 toCarPos(const vec3 newtonPos)
{
    return vec3(newtonPos.x, -newtonPos.z, newtonPos.y);
}

/// Ориентация из координат каркаса в ориентацию тела мира Newton.
Quaternionf toNewtonRot(const Quaternionf carRot)
{
    // Операции dlib над кватернионами не помечены const — работаем на копии.
    Quaternionf r = carToNewtonQuat;
    return r * carRot;
}

/// Кэшированное dagon'ом вращение тела из мира Newton в истинную ориентацию
/// координат каркаса. `body.rotation` — инверсия истинного поворота (см.
/// updateBeamPuppets), поэтому сначала восстанавливаем его `.conj`, затем
/// вычитаем поворот мира — наружу снова уходит геометрия каркаса.
Quaternionf toCarRot(const Quaternionf cachedNewtonRot)
{
    Quaternionf r = carToNewtonQuat;
    Quaternionf c = cachedNewtonRot;
    return r.conj * c.conj;
}

/// Матрица тела в мире Newton из положения и ориентации координат каркаса.
Matrix4x4f newtonBodyMatrix(const vec3 carPos, const Quaternionf carRot)
{
    return translationMatrix(toNewtonPos(carPos)) * toNewtonRot(carRot).toMatrix4x4;
}

/// Ускорение свободного падения мира Newton: вниз вдоль −Y (согласуется
/// с поворотом мира из carToNewtonQuat).
immutable Vector3f gravity = Vector3f(0.0f, -9.80665f, 0.0f);

/// Транспортное состояние тела: позиция и ориентация в координатах машины.
/// Совпадает с трансформацией Dagon-сущности под carRoot:
/// `entity.position = state.position; entity.rotation = state.orientation;`
struct BodyState
{
    Vector3f position;
    Quaternionf orientation;
}

/// Цилиндр Newton создаётся с осью вдоль ЛОКАЛЬНОЙ X, тогда как весь
/// остальной код (и визуальный меш) считает ось цилиндра локальной Y —
/// ось вращения колеса, продольная ось балки. Разворачиваем форму на
/// +90° вокруг Z, чтобы физическая ось легла вдоль локального Y и совпала
/// с мешем: поворот вокруг Z отображает X в Y.
NewtonCylinderShape makeAxisYCylinder(float radius1, float radius2, float height,
    NewtonPhysicsWorld world)
{
    auto shape = New!NewtonCylinderShape(radius1, radius2, height, world);
    shape.setTransformation(rotationQuaternion(Vector3f(0, 0, 1), 0.5f * PI)
        .toMatrix4x4);
    return shape;
}

/// Плоская земля-heightfield для мира Newton: ровная плоскость XZ на y == 0,
/// простирающаяся примерно на `halfExtent` в сторону origin (точный охват
/// задаётся трансформацией тела в buildGround).
///
/// По измерениям Newton 3.14 строит по `size−1` клеток на сторону при аргументе
/// `size` (поле от body-начала на (size−1)·cell), хотя буферы высот и атрибутов
/// ожидает размера size². Поэтому создаём прямоугольник `cells+1 × cells+1`, а
/// из ровной плоскости разница в одну клетку ничего не меняет.
final class GroundHeightfield : NewtonCollisionShape
{
    // Newton 3.14 НЕ копирует height-данные: коллизия хранит указатели на
    // них, поэтому массивы живут всё время жизни коллизии (освобождение —
    // в деструкторе).
    private float[] elevations_;
    private ubyte[] attributes_;

    /// Требуемый полуразмер поля; для вычисления переноса в buildGround.
    float halfExtent;

    this(float halfExtent, uint cells, NewtonPhysicsWorld world)
    {
        super(world);
        this.halfExtent = halfExtent;

        const uint size = cells + 1;
        elevations_ = New!(float[])(size * size);
        foreach (ref h; elevations_)
            h = 0.0f;

        attributes_ = New!(ubyte[])(size * size);
        foreach (ref a; attributes_)
            a = 0;

        const float cell = 2.0f * halfExtent / cast(float)cells;
        newtonCollision = NewtonCreateHeightFieldCollision(world.newtonWorld,
            cast(int)size, cast(int)size, 1, // gridsDiagonals
            0, // elevationdatType: float
            elevations_.ptr, cast(char*)attributes_.ptr,
            1.0f, // verticalScale
            cell, cell, // horizontalScale по X и Z
            0); // shapeId
        NewtonCollisionSetUserData(newtonCollision, cast(void*)this);
    }

    ~this()
    {
        Delete(elevations_);
        Delete(attributes_);
    }
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