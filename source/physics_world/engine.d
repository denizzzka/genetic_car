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

import core.stdc.math : cos, sin, tan;
import std.math : PI;

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

    /// Форма тела: чтобы снятое тело можно было отпустить вместе с формой,
    /// а форма не осталась висеть в мире до его гибели.
    PhysShape shape() @property;

    /// Номер тела в модели (индекс балки, индекс колеса): по нему вердикт
    /// отличает чужое колесо от ступицы своего же якоря.
    void tag(size_t t) @property;
    size_t tag() @property const;

    Vector3f worldPosition() @property;
    Quaternionf worldRotation() @property;

    /// Мировой поворот тела в базисе каркаса. Отличается от `worldRotation`
    /// постоянной поправкой: у фантомного тела локальные оси развёрнуты
    /// относительно каркаса, и без неё поворот уезжает на фиксированный угол.
    Quaternionf worldFrameRotation() @property;
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

/**
 * Труба склеивается из одинаковых брусьев по окружности: описание общее для
 * обоих движков, чтобы покрышка выглядела одинаково.
 */
enum uint tubeSegments = 16;

/// Брус трубы под углом `angle` вокруг оси колеса.
struct TubeSegment
{
    /// Половины габаритов в осях бруса: радиальная, ось колеса, касательная.
    vec3 halfExtent;

    /// Центр бруса в локальных осях колеса.
    vec3 center;
}

TubeSegment tubeSegment(float outerRadius, float innerRadius,
    float height, float angle)
{
    const float rm = 0.5f * (outerRadius + innerRadius);
    const float hr = 0.5f * (outerRadius - innerRadius);
    // По касательной брус длиннее своей дуги: между соседними брусьями иначе
    // остаётся щель, в которую проваливается конец балки.
    const float ht = rm * tan(PI / tubeSegments) * 1.2f;
    TubeSegment segment;
    segment.halfExtent = vec3(hr, 0.5f * height, ht);
    segment.center = vec3(rm * cos(angle), 0.0f, rm * sin(angle));
    return segment;
}

/// Контакт, зарегистрированный за последний шаг.
struct ContactPair
{
    PhysBody a;
    PhysBody b;
}

interface PhysWorld
{
    /// Шаг симуляции. Вызывающий обязан давать `1.0 / updateRate`.
    void step(double dt);

    /// Сколько раз в секунду вызывать `step`. Это знание движка, а не наше:
    /// столько он переваривает за вызов. Внутри движок может дробить dt на
    /// подшаги (у нас обоих их два), так что разрешение контактов выходит
    /// выше этой величины. Вызывающий режет своё время на шаги этой частоты.
    double updateRate() @property;


    /// Считать мир на вызывающем потоке, без побочных потоков движка: заезд
    /// длится один шаг, а внутренние потоки Newton и так живут в пуле.
    void useCallingThread();

    /// Оси базиса в координатах каркаса: каркасные right/forward/up. Бэкенды
    /// переопределяют: внутри оси движка другие, наружу — всегда каркасные.
    immutable vec3 right() @property;
    immutable vec3 forward() @property;
    immutable vec3 up() @property;

    /// Перевод координат и векторов между базисом каркаса и базисом движка.
    vec3 toEnginePos(const vec3 carPos);
    vec3 toCarPos(const vec3 enginePos);
    vec3 toEngineDir(const vec3 carDir);
    vec3 toCarDir(const vec3 engineDir);
    Quaternionf toEngineRot(const Quaternionf carRot);
    Quaternionf toCarRot(const Quaternionf engineRot);

    PhysShape boxShape(const vec3 halfExtent);
    /// Цилиндр с осью вдоль локальной Y — оси мешей модели (продольная
    /// ось балки, ось вращения колеса), независимо от соглашений движка.
    PhysShape cylinderShape(float radius1, float radius2, float height);
    /**
     * Труба: цилиндр с отверстием по оси, ось вдоль локальной Y — как у
     * `cylinderShape`. Покрышка колеса полая, и это не украшение: сквозь
     * ступицу проходит балка подвески, и такой проход не должен считаться
     * контактом, тогда как проход через саму резину — должен.
     */
    PhysShape tubeShape(float outerRadius, float innerRadius, float height);
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

    /// Отпустить форму. Формы, как и тела, живут в списке ownership мира,
    /// поэтому без этого счёта мир копит их до своей гибели; порядок —
    /// сначала снять тело, потом форму.
    void destroyShape(PhysShape shape);

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

/// Создать мир выбранного движка. Само имя движка здесь не упоминается: его
/// выбирает `physics_world.engineselect` (флаг `--engine`), а бэкенды —
/// `physics_world.engine_jolt` и `physics_world.engine_newton` — лежат по
/// сторонам и не знают друг о друге.
static import physics_world.engineselect;

alias createPhysWorld = physics_world.engineselect.createPhysWorld;
