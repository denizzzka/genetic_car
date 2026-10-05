module genetics.growth;

import std.algorithm : max, min, sort;
import std.math : PI, abs, cos, sin, sqrt;
import std.typecons : Nullable;

// `std.math.exp` для float считается портабельным полиномом — в цикле поля
// это втрое дороже libc-варианта.
import core.stdc.math : expf;

import dlib.math.vector : distance, dot, vec3;
import dlib.math.utils : clamp;

import frame.frame : Anchor, AnchorKind, Beam, Frame, Node,
    alongCourse, right;
import physics_world.physics : wheelInnerRadius, wheelWidth;
import physics_world.wheel : defaultWheelRadius;

import frame.cockpit : beamHitsCabin, cockpitFrameContext,
    cockpitFrameBeamCount, cockpitFrameNodeCount, cockpitGeometry,
    cockpitMountNodes, wheelHitsCabin;
import genetics.fitness : maxBeamCount;
import genetics.chromosome : Chromosome;

/*
 * Рост каркаса по Nodal/Lefty.
 *
 * Особи нет никакого плана строения: есть десять генов и правило. Правило
 * такое: в каждом узле каркаса живёт активатор, он тиражирует себя с
 * заданной скоростью и расходится на радиус `actDiffusion`; рядом живёт
 * ингибитор, он тиражируется с `inhProduction`, расходится гораздо дальше
 * (`inhDiffusion`) и гасит всё, до чего дотягивается. Активатор работает
 * локально, ингибитор глобально — в этом весь смысл системы: локально
 * «расти», глобально «не мешать соседям», и получается ветвление, а не шар.
 *
 * Точка, где разность полей превышает порог, получает новую балку. Новая
 * балка сама становится источником, поэтому рост — цепная реакция: каждое
 * поколение балок усиливает поле вокруг себя.
 */

/**
 * Чувствительность тканей к активатору. Ген один, ткань три: каркас слышит
 * поле слабее всего, колесо — сильнее, мотор — сильнее колеса. Разница
 * чувствительности и есть специализация клеток, поэтому моторное колесо
 * всегда подмножество колёс, а каркас — то, что осталось.
 */
enum float[3] tissueGain = [1.0f, 1.15f, 1.45f];
enum size_t frameTissue = 0, wheelTissue = 1, motorTissue = 2;

/// Отклик, при котором ткань отвечает: выше единицы — ткань здесь растёт.
enum float respondGate = 1.0f;

/// Во сколько раз поле мотора должно превосходить порог, чтобы колесо
/// начало крутить. Порог выше единицы: ведущие колёса — редкость, иначе
/// привод появился бы у каждого конца ветви.
enum float motorGate = 0.1f;

/// Потолок числа колёс на машину: больше — уже не средство передвижения,
/// а коллекция.
enum size_t maxWheels = 6;

/// Балок из одного узла: шесть — это крестовина, больше неводоблагообразно.
enum size_t maxNewBeamsPerNode = 6;

/// Узел без родителя: так помечены узлы заданного каркаса кабины.
enum size_t noParent = size_t.max;

/// Зазор между ободами двух колёс, м. Колёса — физические диски: два почти
/// совпадающих обода дают вырожденный контакт в движке, поэтому колесо,
/// которому некуда встать, не ставится вовсе.
enum float wheelGap = 0.02f;

/// Тактов роста: пока есть куда расти, такт повторяется; предел нужен, чтобы
/// вырожденная хромосома не крутила поле вечно.
enum size_t maxGrowthRounds = 40;

/// Слияние: конец балки ближе этого расстояния к чужому узлу не становится
/// новым узлом, а присоединяется к нему балкой. Так ветвь заканчивается
/// петлёй — каркас получает жёсткость, а не цепь висящих концов.
/// Доля масштаба организма: при мелких балках слияние наступает раньше.
enum float mergeShare = 0.35f;

/// Латеральное подавление в пространстве: конец балки ближе этого расстояния к
/// любому чужому узлу не заводится вовсе. Это вторая, чисто геометрическая
/// половина механизма: поле решает, КУДА растёт, а расстояние — КАК далеко
/// ветви могут стоять друг от друга. Доля масштаба организма.
enum float spacingShare = 3.0f;

/// Провисание: балка под подвеской тянется вниз, а вверх растёт плохо. Платит
/// провисание за первые `hangDepth` метров и дальше уже не платит — иначе
/// тяжесть перебивает химию, и организм вытягивается в бесконечную свалю.
/// Глубину ниже подвески задаёт chemistry, а не тяжесть. Это константа
/// механизма — в Lefty сила тяжести тоже не ген, — так что форма остаётся
/// следствием полей.
enum float hangShare = 8.0f;

/// Глубина, на которой провисание перестаёт помогать, м.
enum float hangDepth = 0.45f;

/// Жёсткий предел полуширины машины, м.
enum float maxHalfWidth = 1.1f;

/// Сила короткодействующего подавления вблизи чужой ветви.
enum float crowdShare = 0.4f;

/// Насколько близко к своему предку можно поставить новый узел, не превращая
/// ветвь в виток. Доля масштаба организма.
enum float foldShare = 0.7f;

/// Запас на зазор колеса до чужой балки, м.
enum float beamClearMargin = 0.005f;

/// Насколько поперечная несущая балка повышает отклик колеса. Подвеска, зашедшая
/// сверху, физикой не принимается вовсе, а таких концов в ветви много.
enum float wheelAxleShare = 0.5f;

/// Доля потока, уходящая по курсу.
enum float flowCourseShare = 1.0f;
/// Организатор средней линии: ткань расходится от срединной плоскости.
/// Это и есть тот сигнал, который тянет ветви от кабины в стороны: без него
/// поле самое сильное там, где тело уже есть, и ветви завираются внутрь, а
/// колесо не влезает в промежуток между ними. Симметричен по построению —
/// зависит от расстояния до плоскости, а не от знака.
enum float midlineShare = 16.0f;

/// Ширина, на которой отход от средней линии выходит на насыщение, м.
enum float midlineDepth = 0.5f;

/// Насколько точки считаются одной и той же при поиске зеркальной пары.
/// Больше машинного эпсилона суммирования: у настоящих близнецов координаты
/// совпадают лишь до последнего бита.
enum float mirrorEpsilon = 1e-3f;

/// Длина балки сверх базового шага: насколько её вытягивает локальное поле.
/// Измеряется радиусом действия активатора — это масштаб паттерна, и он же
/// задаёт масштаб его сегментов.
enum float stretchShare = 0.1f;

/// Потолок длины одной балки, м. Гена шага и вытяжки поля недостаточно, чтобы
/// удержать машину в габарите, — предел держит сам рост.
enum float maxBeamLength = 3.0f;

/// Сколько точек внутри балки проверяется на отклик. Поле гладкое (сумма
/// экспонент), поэтому горстки точек хватает: длинная балка не должна
/// перепрыгнуть мёртвый участок.
enum size_t spanSamples = 4;

/// Сколько направлений пробуется из одного конца ветви за такт. Направления
/// берутся с сферы равномерно, поэтому угол ветвления произвольный, а не
/// привязан к шагу сетки.
enum size_t directionProbes = 26;

/// Псевдослучайное число из индекса. Направление роста не должно зависеть от
/// состояния генератора: одна и та же хромосома обязана давать один и тот же
/// каркас в каждом запуске и в каждом потоке.
private uint mixIndex(uint x) pure nothrow @nogc
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

/// Зеркально-каноничный ключ точки: `x` берётся по модулю, иначе точки,
/// зеркальные друг другу, получили бы разные ключи и разные углы.
private uint positionSeed(const vec3 p) pure nothrow @nogc
{
    uint h = cast(uint) cast(int) (abs(p.x) * 1000.0f + 0.5f);
    h += cast(uint) cast(int) (p.y * 1000.0f + 0.5f) * 2654435761u;
    h += cast(uint) cast(int) (p.z * 1000.0f + 0.5f) * 40503u;
    return mixIndex(h);
}

/// Точка сферы по двум равномерным координатам: `z` задаёт высоту, `phi` —
/// азимут. Раскладка равномерная, соседние пробы не слипаются в полюсах.
private vec3 spherePoint(uint seed) pure nothrow @nogc
{
    const uint h = mixIndex(seed);
    const float z = 2.0f * cast(float) (h & 0xFFFFu) / 65535.0f - 1.0f;
    const float phi = 2.0f * PI * cast(float) ((h >> 16) & 0xFFFFu) / 65536.0f;
    const float r = sqrt(max(0.0f, 1.0f - z * z));
    return vec3(r * cos(phi), r * sin(phi), z);
}

/// Состояние одного растущего организма: каркас нарос, поля считаются по нему.
private struct Growth
{
    Chromosome c;
    Frame f;

    /// Узлы заданного каркаса кабины — ниже этого порога ничего не считается
    /// своим: колёса вешаются только на то, что выросло само.
    size_t scaffoldNodes;

    /// Балок, выпущено из каждого узла, — по этому счётчику узел замирает.
    size_t[] sprouts;

    /// Родитель каждого узла: у узлов заданного каркаса — `noParent`. По этой
    /// цепочке рост узнаёт свою ветвь.
    size_t[] parent;

    /// Балок выросло всего.
    size_t grown;

    /// Сумма длин выросших балок — числитель `scale`.
    float grownLen;

    /// Характерный масштаб организма: средняя длина его балок. Запреты роста
    /// (расстояние между ветвями, слияние, виток) меряются им, а не базовым
    /// шагом: вытянувшийся организм иначе ветвит реже, чем велик запрет, и
    /// локальное торможение роста перестаёт его останавливать.
    float scale() const
    {
        return grown > 0 ? max(c.stepLength, grownLen / grown) : c.stepLength;
    }

    /// Потолок для этого организма: общий счётчик каркаса из фитнеса.
    size_t grownMax;

    /// Высота линии подвески: выше неё растёт плохо.
    float hangZ;

    /// Отклик вида в точке: активатор минус ингибитор плюс поток узла и
    /// провисание, делённые на порог. Выше единицы — здесь можно поставить
    /// балку ткани `tissue`.
    ///
    /// `tip` — конец, из которого пойдёт балка: он нужен, чтобы подавление
    /// чужой ветви не съело рост вдоль своей собственной.
    float response(const vec3 p, size_t tissue, size_t tip = noParent) const
    {
        const auto ai = raw(p);
        const float score = tissueGain[tissue] * c.actProduction * ai.act
            - c.inhProduction * ai.inh + flowAt(p) + gravityAt(p)
            + midlineAt(p) - crowdShare * crowding(p, tip);
        return score / c.threshold;
    }

    /// Отход от срединной плоскости: чем дальше от неё, тем сильнее ткань
    /// хочет расти дальше в ту же сторону.
    float midlineAt(const vec3 p) const
    {
        const float dx = abs(p.x - f.nodes[0].pos.x);
        return midlineShare * (1.0f
            - expf(-(dx * dx) / (midlineDepth * midlineDepth)));
    }

    /// Длина новой балки: базовый шаг гена плюс вытяжка, которую даёт
    /// локальное поле. Поле богаче — балка длиннее, и так длина становится
    /// такой же эволюционируемой величиной, как и всё остальное.
    float budLength(const float resp) const
    {
        const float excess = max(0.0f, resp - respondGate);
        return min(c.stepLength + stretchShare * c.actDiffusion * excess,
            maxBeamLength);
    }

    /// Живое ли поле по всей длине балки: длинная балка не перепрыгивает
    /// мёртвый участок, иначе вытяжка стала бы обходом правила роста.
    bool aliveAlong(const vec3 from, const vec3 to) const
    {
        foreach (i; 1 .. spanSamples)
        {
            const float t = cast(float) i / cast(float) spanSamples;
            if (response(from + (to - from) * t, frameTissue) <= respondGate)
                return false;
        }
        return true;
    }

    /// Подавление вблизи чужой ветви: чем ближе точка к чужому выросшему узлу,
    /// тем беднее отклик, и совсем близко рост запрещён.
    ///
    /// Строгий запрет `tooClose` об этом не говорит, а без уклона две ноги
    /// одинаково тянутся к центру корпуса (поле там богаче) и сходятся в
    /// одну точку: запрет не объясняет, куда им расходиться. Мягкий уклон —
    /// это и есть короткодействующий ингибитор, который в Nodal работает на
    /// масштабе ветви, а не на масштабе всего тела.
    float crowding(const vec3 p, size_t tip) const
    {
        if (tip == noParent || f.nodes.length <= scaffoldNodes)
            return 0.0f;
        const float spacing = spacingShare * scale();
        float sum = 0.0f;
        foreach (i; scaffoldNodes .. f.nodes.length)
        {
            // Родной предок не мешает: с ним ветвь растёт вперёд. Остальная
            // своя цепочка мешает — иначе ветвь охотно заворачивает обратно.
            if (i == parent[tip])
                continue;
            const float t = 1.0f - distance(f.nodes[i].pos, p) / spacing;
            if (t > 0.0f)
                sum += t * t;
        }
        return sum;
    }

    /// Сырые поля вида в точке: активатор и ингибитор по отдельности.
    struct Fields
    {
        float act = 0.0f;
        float inh = 0.0f;
    }

    float rawMotor(const vec3 p) const
    {
        float r = 0.0f;
        foreach (n; f.nodes)
        {
            const float d = distance(n.pos, p);
            r += expf(-d / c.motorDiffusion);
        }
        return r * c.motorProduction;
    }

    Fields raw(const vec3 p) const
    {
        Fields r;
        foreach (n; f.nodes)
        {
            const float d = distance(n.pos, p);
            r.act += expf(-d / c.actDiffusion);
            r.inh += expf(-d / c.inhDiffusion);
        }
        return r;
    }

    /// Поток узла: уводит рост вперёд по курсу. Это то же, что течение у
    /// ресничек узла, только у нас оно постоянно и всюду однонаправлено.
    /// Поперечной составляющей нет: она была пропорциональна `rel.x` и
    /// делала поле антисимметричным. Начальная асимметрия, если она нужна,
    /// задаётся снаружи — зеркальным корпусом или стартовой формой.
    float flowAt(const vec3 p) const
    {
        if (c.flowStrength == 0.0f)
            return 0.0f;
        // Только вдоль курса: поперечная составляющая была антисимметричной
        // (пропорциональной `rel.x`) и ломала зеркальность поля. Отход в
        // стороны теперь задаёт organizer средней линии, он симметричен.
        return c.flowStrength * flowCourseShare * alongCourse(p - f.nodes[0].pos);
    }

    /// Провисание в точке: вверх от подвески беднее, вниз — богате, но не
    /// безгранично.
    float gravityAt(const vec3 p) const
    {
        const float d = p.z - hangZ;
        if (d > 0.0f)
            return -hangShare * d;
        const float down = -d;
        return hangShare * down * expf(-down / hangDepth);
    }

    /// Свободный конец: узел ровно с одной балкой. Ветвь растёт концами, и
    /// колесо вешается на конец же — в середине ветви расти некуда.
    bool isTip(size_t node) const
    {
        return degree(node) == 1;
    }

    /// Сколько балок сходятся в узле.
    size_t degree(size_t node) const
    {
        size_t n;
        foreach (b; f.beams)
            if (b.a == node || b.b == node)
                ++n;
        return n;
    }

    /// Одиночная цепочка: колесо вешается только на конец, у которого нет
    /// соседа-брата. У разветвления рядом стоит вторая балка в шаг длиной, и
    /// колесо на таком конце упирается в неё — ровно то, что запрещает фитнес.
    bool onSingleChain(size_t node) const
    {
        return parent[node] == noParent || degree(parent[node]) == 2;
    }

    /// Узел, с которым конец балки сливается: ближайший чужой узел в пределах
    /// порога слияния, или -1.
    ptrdiff_t mergeTarget(const vec3 p, size_t except) const
    {
        const float mergeDist = mergeShare * scale();
        ptrdiff_t best = -1;
        float bestD = mergeDist;
        foreach (i, n; f.nodes)
            if (i != except)
            {
                const float d = distance(n.pos, p);
                if (d < bestD)
                {
                    bestD = d;
                    best = cast(ptrdiff_t) i;
                }
            }
        return best;
    }

    /// Запрещён ли конец балки: он залез в чужую ветвь. Ветви не могут стоять
    /// впритык — это пространственная часть подавления.
    ///
    /// Своя цепочка предков исключена: вдоль себя ветвь растёт свободно, иначе
    /// ей нельзя сделать и двух шагов. Чужим считается только выросшее: заданный
    /// каркас кабины организм обтекает, а не отталкивается от него.
    bool tooClose(const vec3 to, size_t tip) const
    {
        const float spacing = spacingShare * scale();
        const float fold = foldShare * scale();
        foreach (i; scaffoldNodes .. f.nodes.length)
        {
            if (isAncestor(tip, i))
            {
                // Вдоль своей цепочки ветвь растёт свободно — иначе ей нельзя
                // сделать и двух шагов. Обратно в своё же место — нельзя: виток
                // ставит колесо вплотную к балке, на которой оно висит.
                if (i == parent[tip] || distance(f.nodes[i].pos, to) > fold)
                    continue;
                return true;
            }
            if (distance(f.nodes[i].pos, to) < spacing)
                return true;
        }
        return false;
    }

    /// `node` — предок `of` (включая его самого)?
    bool isAncestor(size_t of, size_t node) const
    {
        for (size_t n = of; n != noParent; n = parent[n])
            if (n == node)
                return true;
        return false;
    }

    /// Связаны ли два узла уже существующей балкой: повторная балка между той же
    /// парой узлов не уплотняет каркас, а только тратит потолок сложности.
    bool linked(size_t a, size_t b) const
    {
        foreach (beam; f.beams)
            if ((beam.a == a && beam.b == b) || (beam.a == b && beam.b == a))
                return true;
        return false;
    }

    void addBeam(size_t from, const vec3 to)
    {
        const size_t idx = f.nodes.length;
        f.nodes ~= Node(to);
        parent ~= from;
        const vec3 step = to - f.nodes[from].pos;
        f.beams ~= new Beam(from, idx, c.beamRadius);
        sprouts ~= 0;
        sprouts[from] += 1;
        grown += 1;
        grownLen += sqrt(dot(step, step));
    }
}

/// Почка: конец новой балки, узел-родитель и отклик поля в конце.
private struct Bud
{
    size_t tip;
    vec3 to;

    /// Отклик поля: выше порога — здесь можно расти, и это же мера того,
    /// насколько почка лучшая из доступных.
    float resp;
}

/// Проба направления из конца `tip`: если место годится, почка попадает в
/// список. Габарит проверяется на базовом шаге, поле считаем после него —
/// отбраковка дешёвая.
private void addBud(const Growth g, size_t tip, const vec3 from, const vec3 d,
    ref Bud[] buds)
{
    const vec3 probe = from + d * g.c.stepLength;
    if (abs(probe.x - g.f.nodes[0].pos.x) > maxHalfWidth)
        return;
    const float resp = g.response(probe, frameTissue, tip);
    if (resp <= respondGate)
        return;
    const vec3 to = from + d * g.budLength(resp);
    // Кабина неприкосновенна: сквозь корпус не растём.
    if (beamHitsCabin(g.f, from, to))
        return;
    if (abs(to.x - g.f.nodes[0].pos.x) > maxHalfWidth)
        return;
    buds ~= Bud(tip, to, resp);
}

/// Кандидат на колесо: конец ветви с откликом тканей колеса и мотора.
private struct WheelSpot
{
    size_t node;
    float wheel;
    float motor;
}

/**
 * Хромосома вместе с выращенным каркасом — фенотип, ради которого геном и
 * брался. Носим его между поколениями, чтобы не растить одно и то же дважды.
 */
struct Organism
{
    Chromosome chromosome;
    Nullable!Frame frame;   // null — каркас не вырос
}

/**
 * Вырастить каркас особи: `null`, если колёс вырастить не удалось и машина
 * не поедет принципиально.
 */
Nullable!Frame develop(Chromosome chr)
{
    const auto c = chr.normalized;
    Growth g;
    g.c = c;
    g.f = cockpitFrameContext().frame;
    g.f.motorPower = c.motorPower;
    // Линия подвески — средний уровень бортовых точек крепления.
    float hangSum = 0.0f;
    size_t hangN = 0;
    foreach (i; cockpitMountNodes())
        if (i < g.f.nodes.length)
        {
            hangSum += g.f.nodes[i].pos.z;
            ++hangN;
        }
    g.hangZ = hangN > 0 ? hangSum / cast(float) hangN : g.f.nodes[0].pos.z;
    g.scaffoldNodes = g.f.nodes.length;
    // Бюджет балок есть только у бортовых точек подвески; хребет кабины —
    // уже готовый каркас, из него ничего не растёт.
    g.sprouts = new size_t[g.scaffoldNodes];
    g.sprouts[] = maxNewBeamsPerNode;
    foreach (i; cockpitMountNodes())
        if (i < g.sprouts.length)
            g.sprouts[i] = 0;
    g.parent = new size_t[g.scaffoldNodes];
    g.parent[] = noParent;
    g.grownMax = min(cast(size_t) c.beamBudget, maxBeamCount - cockpitFrameBeamCount());

    foreach (round; 0 .. maxGrowthRounds)
    {
        const bool grew = growRound(g, round);
        if (!grew)
            break;
    }

    placeWheels(g);

    // Машина без колёс не едет: null — сигнал отбору, что хромосома мертва.
    if (g.f.anchors.length < 2)
        return Nullable!Frame.init;
    return Nullable!Frame(g.f);
}

/// Один такт роста: каждый конец выпускает свою лучшую зеркальную пару.
/// `true`, если что-то выросло, — значит, есть смысл сделать ещё такт.
///
/// Отбор локальный, по концам: конец сам решает, куда ему расти, и не делит
/// место с чужими ветвями. Общего списка почек и глобального отбора нет —
/// выбирать, кому достанется место, должен не код, а поле.
private bool growRound(ref Growth g, size_t round)
{
    bool grew;

    // Список концов копируется: новые узлы попадут в работу на следующем
    // такте, иначе одна ветвь уехала бы вперёд за один шаг. Зачаток — свободные
    // концы заданного каркаса кабины: точки крепления и есть те места, где
    // организму дозволено начать.
    // Почка может выйти из любого узла с неисчерпанным запасом, а не только из
    // свободного конца: конец, слившийся с чужой ветвью, иначе гасит рост
    // целиком, а середина ветви всё равно заперта расстоянием до соседей.
    size_t[] tips;
    foreach (i; 0 .. g.f.nodes.length)
        if (g.sprouts[i] < maxNewBeamsPerNode)
            tips ~= i;
    if (tips.length == 0)
        return false;

    // Такт идёт в две фазы. Сначала каждый конец выбирает направление по
    // полю, каким оно было до такта, и решения не записываются в каркас; затем
    // выбранное выращивается. Если выбирать и растить вперемешку, конец,
    // обработанный раньше, успевает изменить поле для соседнего, и зеркальные
    // концы получают разные ответы из-за порядка, а не из-за биологии.
    Bud[] chosen;
    foreach (tip; tips)
    {
        const vec3 from = g.f.nodes[tip].pos;
        // Пробы нумеруются от положения конца, а не от его индекса: у
        // зеркальных концов индексы разные, а поле вокруг них одинаковое, и
        // сеять надо так, чтобы они получили одну и ту же серию углов.
        const uint base = mixIndex(cast(uint) round * 2246822519u
            + cast(uint) g.sprouts[tip] * 40503u
            + positionSeed(from));
        Bud[] buds;
        foreach (k2; 0 .. directionProbes)
        {
            const vec3 d = spherePoint(base + cast(uint) k2 * 97u);
            addBud(g, tip, from, d, buds);
            // Набор проб замыкается на зеркало. Без этого симметричные концы
            // обстреливаются разными углами, и одинаковое поле вокруг них
            // даёт разный результат: симметрия поля не доходит до каркаса.
            addBud(g, tip, from, vec3(-d.x, d.y, d.z), buds);
        }
        // Каждый конец выбирает сам, куда ему расти, и ни с кем не соревнуется:
        // левый не делит место с правым, как и в развитии. Общего списка почек
        // и глобального отбора нет — решать, кому достанется место, должно
        // поле, а не код.
        sort!((a2, b2) => a2.resp > b2.resp)(buds);
        if (buds.length != 0)
            chosen ~= buds[0];
    }

    // Растить тоже парами. Один конец, выросший раньше зеркального, успевает
    // изменить поле, и второй отвечает уже на другом поле — симметрия рассыпается
    // из-за порядка. Проверяем оба места до того, как тронем каркас, и растим
    // только если годны оба: одинокая ветвь без близнеца — рог.
    bool[] taken = new bool[chosen.length];
    foreach (i, bud; chosen)
    {
        if (taken[i])
            continue;
        // Бюджет тела — единственный ограничитель, общий для всех концов: он
        // задаёт размер, а не распределяет место.
        if (g.grown + 2 > g.grownMax)
            break;
        const auto twin = findMirror(chosen, i);
        if (twin < 0)
            continue;
        if (!canGrow(g, bud) || !canGrow(g, chosen[twin]))
            continue;
        commitBud(g, bud);
        commitBud(g, chosen[twin]);
        taken[i] = true;
        taken[twin] = true;
        grew = true;
    }
    return grew;
}

/// Годится ли место под балку: поле не выжжено, span живой, рядом нет ни
/// чужого узла, ни своего предка.
private bool canGrow(const Growth g, const Bud bud)
{
    if (g.response(bud.to, frameTissue) <= respondGate)
        return false;
    if (!g.aliveAlong(g.f.nodes[bud.tip].pos, bud.to))
        return false;
    const auto merged = g.mergeTarget(bud.to, bud.tip);
    if (merged >= 0)
    {
        // Балка к узлу-слиянию уже есть — расти тут больше нечего, а новый
        // узел в чужой точке поставил бы дубль и вторую копию той же балки.
        return !g.linked(bud.tip, cast(size_t) merged);
    }
    return !g.tooClose(bud.to, bud.tip);
}

/// Врастить почечную балку, минуя слияние: место уже проверено, балка ляжет
/// новым узлом.
private void commitBud(ref Growth g, const Bud bud)
{
    const auto merged = g.mergeTarget(bud.to, bud.tip);
    if (merged < 0)
    {
        g.addBeam(bud.tip, bud.to);
        return;
    }
    g.f.beams ~= new Beam(bud.tip, cast(size_t) merged, g.c.beamRadius);
    g.sprouts[bud.tip] += 1;
    g.grown += 1;
    g.grownLen += distance(g.f.nodes[bud.tip].pos,
        g.f.nodes[cast(size_t) merged].pos);
}

/// Зеркальный близнец почки `i` среди остальных выбранных. Ищем именно среди
/// концов: у самого конца зеркальные направления недопустимы — они уводят в
/// кабину, — так что пара есть только между разными концами.
private ptrdiff_t findMirror(const Bud[] buds, size_t i)
{
    const vec3 want = vec3(-buds[i].to.x, buds[i].to.y, buds[i].to.z);
    ptrdiff_t found = -1;
    float best = mirrorEpsilon;
    foreach (j; 0 .. buds.length)
    {
        if (j == i)
            continue;
        const float d = distance(want, buds[j].to);
        if (d <= best)
        {
            best = d;
            found = cast(ptrdiff_t) j;
        }
    }
    return found;
}

/**
 * Колёса вырастают сами: свободный конец ветви становится колесом там, где
 * поле колеса отвечает, и моторным — где поле мотора отвечает вдвое сильнее
 * порога. Никаких пар, зеркал и заранее заданных мест: сколько колёс выросло
 * и какие из них ведущие — следствие роста, а не решение грамматики.
 *
 * Если моторного поля нигде не хватило, ведущим становится сильнейшее колесо:
 * неподвижная машина не отбирается ни с чем.
 */
private void placeWheels(ref Growth g)
{
    WheelSpot[] spots;
    foreach (i; g.scaffoldNodes .. g.f.nodes.length)
    {
        if (!g.isTip(i) || !g.onSingleChain(i))
            continue;
        const vec3 p = g.f.nodes[i].pos;
        // Ось колеса поперечная, и подвеска должна заходить по ней: балка
        // поперёк идёт по оси, внутри ступицы, и резину не трогает. Отсюда
        // бонус концу, чья несущая балка поперечна.
        const float wheel = g.response(p, wheelTissue)
            + wheelAxleShare * axleShare(g, i);
        if (wheel <= respondGate || wheelHitsCabin(g.f, p, g.c.wheelRadius))
            continue;
        spots ~= WheelSpot(i, wheel, g.rawMotor(p));
    }
    sort!((a, b) => a.wheel > b.wheel)(spots);

    bool motor;
    foreach (spot; spots)
    {
        if (g.f.anchors.length >= maxWheels)
            break;
        if (!wheelFits(g, spot.node))
            continue;

        const bool driven = spot.motor > motorGate;
        motor = motor || driven;
        g.f.anchors ~= Anchor(spot.node,
            driven ? AnchorKind.motorWheel : AnchorKind.wheel, g.c.wheelRadius);
    }
    if (!motor && g.f.anchors.length > 0)
        g.f.anchors[0].kind = AnchorKind.motorWheel;
}

/// Влезает ли колесо на этот конец.
///
/// Три ограничения, и все они — геометрия, а не договорённость:
///   - обод не наезжает на уже поставленные колёса;
///   - ни одна чужая балка не задевает покрышку (свою несущую не считаем: ось
///     колеса легитимно проходит через ступицу, это и есть подвеска);
///   - ось колеса выше земли хотя бы на половину радиуса, иначе колесо сразу
///     уходит под грунт.
private bool wheelFits(const Growth g, size_t node)
{
    const vec3 p = g.f.nodes[node].pos;
    const float r = g.c.wheelRadius;

    foreach (a; g.f.anchors)
        if (distance(p, g.f.nodes[a.node].pos) < r + a.radius + wheelGap)
            return false;

    return !beamHitsTyre(g, node);
}

/// Насколько несущая балка конца поперечна: 1 — строго по оси колеса.
private float axleShare(const Growth g, size_t tip)
{
    foreach (b; g.f.beams)
        if (b.a == tip || b.b == tip)
        {
            const vec3 other = g.f.nodes[b.a == tip ? b.b : b.a].pos
                - g.f.nodes[tip].pos;
            const float len = sqrt(dot(other, other));
            return len > 0.0f ? abs(dot(other / len, right)) : 0.0f;
        }
    return 0.0f;
}

/// Задевает ли чужая балка покрышку.
///
/// Покрышка — труба вокруг поперечной оси (оси колеса всегда поперечные, см.
/// `wheelAxle`), поэтому у точки есть «внутрь-от-оси» и «вдоль оси». Задевает
/// ровно то, что попало в радиальную полосу между ступицей и ободом И в
/// ширину покрышки. Балка, идущая вдоль оси, в полосу не попадает: она идёт
/// внутри ступицы, поэтому подвеска, зашедшая сбоку, резину не трогает, а
/// зашедшая сверху проходит через неё.
private bool beamHitsTyre(const Growth g, size_t node)
{
    const vec3 hub = g.f.nodes[node].pos;
    const float scale = g.c.wheelRadius / defaultWheelRadius;
    const float inner = wheelInnerRadius * scale - g.c.beamRadius;
    const float outer = g.c.wheelRadius + g.c.beamRadius;
    const float halfWidth = 0.5f * wheelWidth * scale + g.c.beamRadius;

    bool hits(const vec3 q)
    {
        // Радиус до оси колеса: поперечная составляющая, ось уходит в x.
        const float radial = sqrt(q.y * q.y + q.z * q.z);
        return abs(q.x) <= halfWidth && radial >= inner && radial <= outer;
    }

    foreach (b; g.f.beams)
    {
        if (b.a == node || b.b == node)
            continue;
        const vec3 pa = g.f.nodes[b.a].pos - hub;
        const vec3 pb = g.f.nodes[b.b].pos - hub;
        if (hits(pa) || hits(pb))
            return true;
        // Точка балки, ближайшая к оси: у неё радиус минимален, и если она
        // в полосе, то в полосе и вся балка.
        const float dx = pb.x - pa.x;
        if (abs(dx) > 1e-6f)
        {
            const float t = clamp(-pa.x / dx, 0.0f, 1.0f);
            if (hits(pa + (pb - pa) * t))
                return true;
        }
    }
    return false;
}

private float pointSegmentDistance(const vec3 p, const vec3 a, const vec3 b)
{
    const vec3 ab = b - a;
    const float len2 = dot(ab, ab);
    if (len2 < 1e-12f)
        return distance(p, a);
    const float t = clamp(dot(p - a, ab) / len2, 0.0f, 1.0f);
    return distance(p, a + ab * t);
}

unittest
{
    import frame.frame : isConnected;
    import genetics.fitness : buggyFitness;

    // Основатель вида обязан вырасти в машину, а не в комок: каркас связный,
    // есть колёса и хотя бы одно ведущее.
    const Chromosome founder;
    const auto grown = develop(founder);
    assert(!grown.isNull, "основатель обязан вырасти в машину с колёсами");

    const auto f = grown.get;
    assert(f.beams.length > cockpitFrameBeamCount(), "основатель обязан что-то дорастить");
    assert(isConnected(f), "рост не оставляет висячих узлов");
    assert(f.anchors.length >= 2, "машине нужно хотя бы два колеса");
    bool motor;
    foreach (a; f.anchors)
        if (a.kind == AnchorKind.motorWheel)
            motor = true;
    assert(motor, "без ведущего колеса машина не едет");
    assert(f.motorPower == founder.motorPower, "сила мотора — из гена");
    assert(buggyFitness(f) > 0.0f, "основатель обязан быть жизнеспособен");
}