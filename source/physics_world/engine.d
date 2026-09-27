/**
 * Граница физики: контракты, о которых говорит модель машины.
 *
 * Движок не знает ничего ни о раме, ни о колёсах, ни о вердиктах: он
 * создаёт тела по логическим ролям, решает контакты и сообщает, какие пары
 * ролей столкнулись. Всё остальное — геометрия, массы, привод, вердикты —
 * живёт в модели и не зависит от выбора движка.
 *
 * Координаты везде car-локальные: X вправо, Y вперёд, Z вверх. Перевод в
 * оси движка (у Newton «вверх» вдоль Y) — забота бэкенда, наружу он не
 * выходит: иначе переключение движка задевало бы каждую формулу модели.
 */
module physics_world.engine;

import dlib.core.ownership;
import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;

/// Логическая роль тела в машине. Роль — единственное, что движок знает о
/// назначении тела; вердикт по контакту модель выносит сама.
enum BodyRole
{
    /// Плоскость или стриминговое окно terrain. Тела не существует.
    ground,
    /// Единый центр масс и инерции рамы, к которому приварены колёса.
    master,
    /// Динамическое колесо на своей оси.
    wheel,
    /// Балка каркаса: контакт виден, но никогда не толкает тело.
    beam,
    /// Статическое препятствие в мире (граница зоны): ловит машину, но к
    /// каркасу не принадлежит.
    obstacle,
}

/// Как движок ведёт тело.
enum BodyMotion
{
    staticBody,
    kinematicBody,
    dynamicBody,
}

/// Что контакт двух ролей значит для машины.
enum ContactResponse
{
    /// Движок отбрасывает пару: узел каркаса в точке — не столкновение.
    ignore,
    /// Обычное решение контакта.
    resolve,
    /// Решить контакт и доложить паре модели. Провал заезда коллизию не
    /// отменяет: тела всё равно должны разойтись честно.
    resolveAndReport,
}

/// Правило контакта между двумя ролями. Сцепление и упругость значимы только
/// для `resolve`/`resolveAndReport`; `ignore` их не читает.
struct Interaction
{
    BodyRole a;
    BodyRole b;
    ContactResponse response = ContactResponse.resolve;
    /// Порог срыва (статическое) и удержание при скольжении (кинетическое):
    /// Newton различает их, и разница видна на разгоне.
    float friction = 0.5f;
    float kineticFriction = 0.5f;
    float elasticity = 0.0f;
}

interface PhysShape
{
}

/// Тело физического мира. Трансформы задаются и читаются в car-координатах.
interface PhysBody
{
    PhysWorld physWorld() @property;
    BodyRole role() @property const;

    /// Номер тела в модели (индекс балки, индекс колеса): по нему вердикт
    /// отличает чужое колесо от ступицы своего же якоря.
    void tag(size_t t) @property;
    size_t tag() @property const;

    Vector3f worldPosition() @property;
    Quaternionf worldRotation() @property;
    /// Поворот вокруг текущего центра масс: геттеры обязаны видеть результат
    /// сразу, без шага симуляции.
    void worldRotation(Quaternionf carRot) @property;
    void worldTransform(const vec3 carPos, const Quaternionf carRot) @property;
    /// Поставить тело в точку, не трогая ориентацию: у земли она не важна,
    /// а поворот «в единичную» в координатах каркаса — уже поворот в осях
    /// движка, и земля встала бы с наклоном.
    void setWorldPosition(const vec3 carPos);

    Vector3f velocity() @property;
    void velocity(Vector3f v) @property;
    Vector3f angularVelocity() @property;
    void angularVelocity(Vector3f w) @property;

    Vector3f worldCenterOfMass() @property;
    /// Смещение ЦМ в локальных осях тела: локальный вектор переводить в оси
    /// движка нельзя, иначе ось колеса съедет на борт.
    void localCenterOfMass(Vector3f offset) @property;
    void mass(float m) @property;
    /// Главные моменты инерции тела в его локальных осях.
    void inertia(Vector3f principal) @property;
    /// Модуль ускорения свободного падения тела. Направление «вниз» знает
    /// движок: наружу отдаётся только модуль, ось у каждого движка своя.
    void gravity(float accel) @property;
    void linearDamping(float damping) @property;
    /// Покомпонентное демпфирование в локальных осях тела.
    void angularDamping(Vector3f damping) @property;
    /// Выключить коллизию, оставив тело носителем массы (рама-мост).
    void collidable(bool on) @property;

    void addForce(Vector3f f);
    void addTorque(Vector3f t);

    /// Протолкнуть новую позу в движок: без этого кинематическое тело
    /// держит прежний кэш трансформации до следующего `step`.
    void syncPose();
}

interface PhysJoint
{
    PhysBody bodyA() @property;
    PhysBody bodyB() @property;
}

/// Ось колеса: держит ступицу на раме, свободно оставляя только вращение.
interface PhysAxleJoint : PhysJoint
{
}

/// Рулевой рычаг: шарнир с приводом на рыскание относительно рамы.
interface PhysSteerJoint : PhysAxleJoint
{
    void targetYaw(float yaw) @property;
    float targetYaw() @property const;
    float yawNow() @property const;
}

/// Контакт, зарегистрированный за последний шаг.
struct ContactPair
{
    PhysBody a;
    PhysBody b;
}

interface PhysWorld
{
    /// Шаг симуляции.
    void step(double dt);

    /// Считать мир на вызывающем потоке, без пула: заезд длится один шаг,
    /// а пул физики уже держит зуботистую очередь Newton.
    void useCallingThread();

    PhysShape boxShape(const vec3 halfExtent);
    /// Цилиндр с осью вдоль локальной Y — оси мешей модели (продольная
    /// ось балки, ось вращения колеса), независимо от соглашений движка.
    PhysShape cylinderShape(float radius1, float radius2, float height);
    /// Статическая геометрия из треугольников в car-координатах: вершины,
    /// нормали к ним и индексы по три на грань.
    PhysShape triangleMeshShape(const float[] vertices, const float[] normals,
        const uint[] indices);

    /// Ровная земля с верхом на `topZ`, занимающая `halfExtent` в сторону
    /// начала координат.
    PhysBody createFlatGround(float halfExtent, uint cells, float topZ);
    /// Окно terrain: сетка высот `heights` размером `dims × dims` в car-
    /// координатах с шагом `cell`, угол мира в `origin` (левый нижний угол).
    PhysBody createHeightfieldGround(const float[] heights, uint dims, float cell,
        const vec3 origin);

    PhysBody createBody(BodyRole role, BodyMotion motion, PhysShape shape,
        float mass);

    /// Снести одно тело, оставив мир и его настройки: окно terrain
    /// пересобирается на каждом смене центрального тайла.
    void destroyBody(PhysBody body);

    PhysAxleJoint newAxleJoint(PhysBody wheel, PhysBody master, const vec3 pivotMaster);
    /// `limit` — предел хода рыскания, `errorCap` — потолок подпитки ряда:
    /// без него ненагруженная балка раскручивает сама себя.
    PhysSteerJoint newSteerJoint(PhysBody steer, PhysBody master,
        const vec3 pivotSteer, const vec3 pivotMaster, const vec3 armUp,
        float limit, float errorCap);

    void setInteraction(in Interaction rule);

    /// Контакты, зарегистрированные за последний шаг: ровно те, на которые
    /// модель подписана, — `ignore` движок не доносит.
    const(ContactPair)[] contacts() const;

    /// Сбросить мир в пустое состояние для следующего заезда: тела и контакты
    /// уходят, настройки групп и взаимодействий остаются.
    void clearScene();

    /// Освободить мир целиком. Пул отдаёт миру жизнь обратно, поэтому
    /// `clearScene` здесь недостаточно.
    void dispose();
}

/// Создать мир выбранного движка — единственное место, где модель знает про
/// бэкенд. Подмена движка это другая реализация `PhysWorld` плюс правка
/// этой строки.
static import physics_world.engine_newton;

alias createPhysWorld = physics_world.engine_newton.createPhysWorld;
