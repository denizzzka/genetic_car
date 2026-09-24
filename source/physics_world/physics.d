module physics_world.physics;

import std.math;
import std.algorithm : min;
import std.exception : enforce;

import dlib.core.memory;
import dlib.core.ownership;
import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;
import dlib.math.transformation;

import dagon.core.event;
import dagon.ext.newton;

import frame.frame : Frame, Node, Beam, Anchor, AnchorKind, origin,
    right, up, forward;

/// Радиус колеса по умолчанию, м. Совпадает с внешним радиусом визуального
/// тора и коллизионного цилиндра. Генетический радиус каждого колеса
/// масштабируется относительно этого базового: внутренний радиус и ширина
/// покрышки растут/сжимаются пропорционально.
enum float wheelRadius = 0.3f;

/// Предельная линейная скорость поверхности колеса (обода), м/с. Крутка
/// быстрее гасится мягким регулятором в `BuggyPhysics`.
enum float wheelMaxSurfaceSpeed = 75.0f / 3.6f;

/// Предельная угловая скорость вращения колеса, рад/с: лимит скорости
/// обода `wheelMaxSurfaceSpeed` при базовом радиусе `wheelRadius`. Для
/// генетического размера колеса предел пересчитывается как
/// `wheelMaxSurfaceSpeed / radius`.
enum float wheelOmegaMax = wheelMaxSurfaceSpeed / wheelRadius;

/// Жёсткость регулятора крутки, Н·м на рад/с превышения лимита. Чем
/// больше — тем плотнее предел, но и тем резче торможение сверх него.
/// 50 устойчиво при dt = 1/60 и осевом моменте инерции колеса ~0.72 кг·м²
/// (предел дискретной P-петли gain < 2I/dt ≈ 86).
enum float wheelSpinGain = 50.0f;

/// Максимальное угловое ускорение колеса, рад/с²: любой момент (привод и
/// антипробуксовка) ограничивается I·α_max. Ограничение приземляет
/// двигателю разгон малого колеса, чью инерцию (порядка 10⁻³–10⁻¹ кг·м² при
/// генетическом радиусе ниже базового) сцепление покрышки с грунтом не
/// сдержать: без предела момент мотора за один шаг закручивает колесо на
/// тысячи рад/с и разносит каркас. Для штатного колеса потолок I·α_max
/// ≈ 2200 Н·м — момент мотора его не касается, поведение не меняется.
enum float maxWheelAngularAccel = 3000.0f;

/// Доля границы устойчивости дискретной P-петли регулятора крутки
/// (gain < 2I/dt): во столько раз ниже предела держится его агрессивность.
/// Масштабируя gain по осевой инерции колеса, регулятор остаётся стабильным
/// и для малых колёс — иначе фиксированный `wheelSpinGain` в сотни раз выше
/// границы 2I/dt качает колесо вдвое сильнее на каждом шаге (расходимость
/// даёт те же разлёты каркаса).
enum float wheelSpinGainMargin = 0.75f;

/// Колесо глубже этой отметки относительно земли — провал сквозь неё.
enum float physicsWheelBelow = -0.1f;

/// Допустимое проскальзывание колеса относительно грунта, в долях скорости
/// грунта под ним. Буксование ниже этого допуска ничем не стесняется —
/// предел мягкий. 0.15 — заметный букс на разгоне, но не разнос воздуха.
enum float wheelSlipRatio = 0.15f;

/// Стартовое окно скорости обода, м/с: пока машина стоит (скорость грунта
/// под ней ~0), обод всё равно может раскручиваться до этой скорости,
/// иначе тронуться с места было бы нечем — при нулевом проскальзывании
/// колесо не толкает. Позволяет тронуться с холма и со свежей точки.
enum float wheelLaunchSurfaceSpeed = 15.0f / 3.6f;

/// Наклон рамы (крен или тангаж) больше этой величины относительно стартовой
/// вертикали — переворот. Считается по вектору «вверх» рамы, а не по высоте
/// ступицы: подъём колеса на крене или отрыве переворотом не является.
enum float maxTiltDegrees = 40.0f;

/// Застой по курсу: машина, которая столько секунд симуляции не набирает
/// нового продвижения вперёд по курсу (-Y каркаса), сходит с дистанции.
enum double stallSeconds = 30.0;

/// Чистый набег вперёд по курсу, обнуляющий таймер застоя: дрожание и
/// качание на месте (< порога) продвижением не считаются.
enum float stallProgressEps = 0.1f;

/// Высота броска на старт: низ самого низкого колеса ставится на эту высоту
/// над землёй, дальше физика сама роняет багги на поверхность.
enum float dropHeight = 0.15f;

/// Раскладка каркаса «на старт»: центрирует горизонтально (средняя X/Y узлов —
/// в ноль) и сажает низом самого низкого колеса на `dropHeight` над землёй —
/// багги роняют на поверхность при старте. Единый способ поставить машину —
/// им пользуются и грамматика (`develop`), и физика (`BuggyPhysics`), и витрина.
/// Низ каждого колеса считается по его собственному генетическому радиусу.
vec3 placeOffset(const Frame f)
{
    vec3 c = origin;
    foreach (n; f.nodes)
        c += n.pos;
    if (f.nodes.length > 0)
        c /= f.nodes.length;

    float minBottom = float.max;
    foreach (a; f.anchors)
        minBottom = min(minBottom, f.nodes[a.node].pos.z - a.radius);
    const float dz = (minBottom < float.max) ? dropHeight - minBottom : 0.0f;

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

/// Внутренний радиус «отверстия» колеса (покрышка — полый цилиндр) при
/// базовом радиусе `wheelRadius`. Нужен только для массы и тензора инерции;
/// коллизия — по внешней поверхности сплошного цилиндра, как и у исходной
/// опорной функции. Для генетического радиуса масштабируется пропорционально.
enum float wheelInnerRadius = 0.22f;

/// Ширина колеса (длина оси цилиндра, равна внешней толщине тора) при
/// базовом радиусе `wheelRadius`. Масштабируется пропорционально радиусу
/// генетического колеса.
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

/// Сцепление пары «покрышка × грунт» (сухая почва/глина): статическое —
/// срыв (зацепление), кинетическое — удержание при скольжении.
enum float soilFrictionStatic = 0.7f;
enum float soilFrictionKinetic = 0.55f;

/// Упругость пары «покрышка × грунт»: мягкая покрышка гасит удар о почву,
/// отскок мал (ближе к мокрой глине, ~0.05).
enum float soilElasticity = 0.05f;

/// Сцепление пары «колесо × колесо»: контакт колёс — провал заезда, пару
/// решать не нужно, коэффициент — прежний 0.9, чтобы поведение не менялось.
enum float wheelWheelFriction = 0.9f;
enum float wheelWheelElasticity = 0.0f;

/// Общее демпфирование рамы (мастер) и перпендикулярных оси спина осей
/// колёс: прежнее наследственное значение.
enum float bodyDamping = 0.5f;

/// Линейное демпфирование колёс: лёгкий выкат по грунту (паразитные потери
/// в покрышке) — снижено с 0.5, тормозит сцепление, а не «воздух».
enum float tireLinearDamping = 0.1f;

/// Сопротивление качению (Crr ~ 0.1 по грунту): угловое демпфирование оси
/// спина колеса — обод выкатывается короче, чем на асфальте.
enum float tireRollDamping = 0.2f;

/// Контакты балок (sensor) с грунтом разбираются как у dagon для sensor×default:
/// все точки снимаются, чтобы кинестатическая балка не толкалась землёй.
/// Сигнала на провал не нужно — раму под землёй ловит геометрическая
/// beamUnderground().
extern(C) void physicsSensorGroundContacts(
    const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
    void* next;
    for (void* c = NewtonContactJointGetFirstContact(contactJoint); c; c = next)
    {
        next = NewtonContactJointGetNextContact(contactJoint, c);
        NewtonContactJointRemoveContact(contactJoint, c);
    }
}

/// Мир Newton с подписанной группой материала грунта. Грунт (почва/глина)
/// получает собственную группу, чтобы сцепление покрышек бралось только с
/// него (пара default×soil), а пара колесо×колесо (default×default) и всё
/// прочее осталось как в dagon из коробки.
final class PhysicsWorld : NewtonPhysicsWorld
{
    /// Материальная группа грунта: в ней живут тела земли (плоскость и
    /// террейн). Колёса катятся по ней с offroad-сцеплением.
    int soilGroupId;

    this(EventManager eventManager, Owner o)
    {
        super(eventManager, o);
        soilGroupId = createGroupId();

        // Сцепление и упругость — только пара «покрышка × грунт».
        NewtonMaterialSetDefaultFriction(newtonWorld, defaultGroupId, soilGroupId,
            soilFrictionStatic, soilFrictionKinetic);
        NewtonMaterialSetDefaultElasticity(newtonWorld, defaultGroupId, soilGroupId,
            soilElasticity);
        // Балки (sensor) по грунту — разбор контактов без толкания.
        NewtonMaterialSetCollisionCallback(newtonWorld, sensorGroupId, soilGroupId,
            null, &physicsSensorGroundContacts);
    }
}

/// Группа материала грунта мира (см. `PhysicsWorld`). Все миры приложения
/// создаются как `PhysicsWorld` (пул и одиночные заезды), поэтому каст безопасен.
int soilGroupIdOf(const NewtonPhysicsWorld world)
{
    auto pw = cast(PhysicsWorld) world;
    assert(pw !is null, "мир обязан создаваться как PhysicsWorld");
    return pw.soilGroupId;
}

/// Ориентация физического мира: Newton держит «вверх» вдоль своей Y (земля —
/// плоскость XZ, у heightfield'а ось высоты — Y), а у каркаса и вьюера
/// «вверх» — Z. Мир Newton — это геометрия каркаса, повёрнутая вокруг X на
/// −90°: (x, y, z)каркас → (x, z, −y)newton. Все положения и ориентации тел,
/// уходящие в Newton и возвращающиеся из него, проходят через переводы ниже.
/// Значение выведено из образов осей базиса (`carToNewtonBasis`): литерал —
/// потому что LDC сворачивает `fromMatrix` в NaN на этапе компиляции.
immutable Quaternionf carToNewtonQuat =
    Quaternionf(-0.70710678f, 0.0f, 0.0f, 0.70710678f);

/// Собирает поворот каркас→Newton из матрицы образов осей базиса. Образы осей
/// проверяет юнит-тест: right→правый X, up→верхний Y, forward→курс Z.
private Quaternionf carToNewtonBasis()
{
    Matrix4x4f fromBasis = Matrix4x4f([
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -1.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f]);
    return Quaternionf.fromMatrix(fromBasis);
}

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

unittest
{
    // Раскладка на старт учитывает генетический радиус каждого колеса: низ
    // определяется самым нижним ободом (центр минус радиус), а не позицией
    // центра — большое колесо поднимает раму выше.
    Frame f;
    f.nodes = [Node(origin), Node(vec3(0.0f, 1.0f, 0.0f))];
    f.beams = [Beam(0, 1, 0.05f)];
    // Большое колесо (радиус 0.75 м) у узла 0, маленькое (0.05 м) у узла 1.
    f.anchors = [
        Anchor(0, AnchorKind.wheel, 0.75f),
        Anchor(1, AnchorKind.wheel, 0.05f),
    ];

    const off = placeOffset(f);
    // Нижний обод большого колеса: 0 - 0.75 = -0.75; подъём — на dropHeight.
    assert(abs(off.z - (dropHeight + 0.75f)) < 1e-5f,
        "раскладка садит низ большого колеса на dropHeight");
    assert(abs((f.nodes[0].pos.z + off.z) - 0.75f - dropHeight) < 1e-5f,
        "нижняя точка большого колеса оказывается над землёй ровно на dropHeight");
    assert(abs((f.nodes[1].pos.z + off.z) - 0.05f - dropHeight) > 0.5f,
        "маленькое колесо парит над землёй — оно не задаёт плоскость старта");

    // Сравнение с прежним поведением: колёса одного базового радиуса.
    Frame g;
    g.nodes = [Node(origin), Node(vec3(0.0f, 1.0f, -0.2f))];
    g.beams = [Beam(0, 1, 0.05f)];
    g.anchors = [
        Anchor(0, AnchorKind.wheel),
        Anchor(1, AnchorKind.motorWheel, 0.3f),
    ];
    const offOld = placeOffset(g);
    float minZ = float.max;
    foreach (a; g.anchors)
        minZ = min(minZ, g.nodes[a.node].pos.z);
    assert(abs(offOld.z - (wheelRadius + dropHeight - minZ)) < 1e-5f,
        "радиус по умолчанию сохраняет прежнюю раскладку");
}

unittest
{
    // Поворот каркас→Newton задаёт образы осей базиса: right остаётся правым,
    // up становится up Newton, forward — курсом Newton. Литерал хранится
    // отдельно (см. `carToNewtonQuat`) — сверяем его через базис.
    Quaternionf q = carToNewtonBasis();
    const vec3 rx = q.rotate(right);
    const vec3 ry = q.rotate(up);
    const vec3 rz = q.rotate(forward);
    assert(abs(rx.x - 1.0f) < 1e-5f && abs(rx.y) < 1e-5f && abs(rx.z) < 1e-5f,
        "right каркаса остаётся правым в Newton");
    assert(abs(ry.x) < 1e-5f && abs(ry.y - 1.0f) < 1e-5f && abs(ry.z) < 1e-5f,
        "up каркаса становится up Newton");
    assert(abs(rz.x) < 1e-5f && abs(rz.y) < 1e-5f && abs(rz.z - 1.0f) < 1e-5f,
        "forward каркаса становится курсом Newton");
}

unittest
{
    // Группа материала грунта — отдельная подписанная группа, а не магическое
    // число: не совпадает со стандартными default/sensor/kinematic мира.
    ensureNewtonLoaded();
    auto w = New!PhysicsWorld(cast(EventManager)null, cast(Owner)null);
    scope (exit) Delete(w);
    assert(w.soilGroupId == soilGroupIdOf(w),
        "доступ к группе грунта через хелпер совпадает с полем");
    assert(w.soilGroupId > 0 && w.soilGroupId != w.defaultGroupId
        && w.soilGroupId != w.sensorGroupId
        && w.soilGroupId != w.kinematicGroupId,
        "грунт живёт в отдельной группе материалов");
}
