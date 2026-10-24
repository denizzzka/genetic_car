module genetics.growth;

import std.algorithm : canFind, max, min, sort;
import std.math : PI, abs, cos, round, sin, sqrt;
import std.typecons : Nullable;

// `std.math.exp` для float считается портабельным полиномом — в цикле поля
// это втрое дороже libc-варианта.
import core.stdc.math : expf;

import dlib.math.vector : distance, dot, vec3;
import dlib.math.utils : clamp;
import core.sync.mutex;

import frame.frame : Anchor, AnchorKind, Beam, Frame, Node, forward, up,
    alongCourse, right;
import physics_world.physics : beamHitsTyre, wheelWidth;
import physics_world.wheel : defaultWheelRadius, wheelAxle;

import frame.cockpit : beamHitsCabin, cockpitFrameContext,
    cockpitFrameBeamCount, cockpitFrameNodeCount, cockpitGeometry,
    cockpitMountNodes, wheelHitsCabin;
import genetics.fitness : maxBeamCount;
import genetics.chromosome : Chromosome, SegmentGenes, normalized;

/// Версия правил роста: меняется при правках констант или алгоритма.
enum uint growthGrammarVersion = 1;

/*
 * Рост каркаса по Nodal/Lefty.
 *
 * Особи нет никакого плана строения: есть гены и правило. Правило такое: в
 * каждом узле каркаса живёт активатор, он тиражирует себя с заданной скоростью
 * и расходится на радиус `actDiffusion`; рядом живёт ингибитор, он тиражируется
 * с `inhProduction`, расходится гораздо дальше (`inhDiffusion`) и гасит всё, до
 * чего дотягивается. Активатор работает локально, ингибитор глобально — в этом
 * весь смысл системы: локально «расти», глобально «не мешать соседям», и
 * получается ветвление, а не шар.
 *
 * Гены поля читаются пофазно: хромосома — список фаз, и каждая доращивает
 * каркас своим полем. Стойка руля, поперечина и корпус — не частные случаи
 * кода, а последовательные фазы одного правила.
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
enum float mergeShare = 0.055f;

/// Латеральное подавление в пространстве: конец балки ближе этого расстояния к
/// любому чужому узлу не заводится вовсе. Это вторая, чисто геометрическая
/// половина механизма: поле решает, КУДА растёт, а расстояние — КАК далеко
/// ветви могут стоять друг от друга. Доля масштаба организма.
enum float spacingShare = 0.47f;

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
enum float foldShare = 0.11f;

/// Запас на зазор колеса до чужой балки, м.
enum float beamClearMargin = 0.005f;

/// Насколько поперечная несущая балка повышает отклик колеса. Подвеска, зашедшая
/// сверху, физикой не принимается вовсе, а таких концов в ветви много.
enum float wheelAxleShare = 0.5f;

/// Доля потока, уходящая по курсу.
enum float flowCourseShare = 1.0f;

/// Ширина, на которой отход от средней линии выходит на насыщение, м.
enum float midlineDepth = 0.15f;

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

/// Ключ серии углов для конца: его собственное состояние, а не координата на
/// каркасе. Направление берётся в базисе тела, поэтому у зеркальных концов
/// оно совпадает, хотя в мире противоположно, и обе серии углов выходят
/// зеркальными. Число отростков и номер такта — тоже локальные величины.
private uint budSeed(size_t round, size_t sprouts, const vec3 bornLocal)
    pure nothrow @nogc
{
    uint h = cast(uint) cast(int) (bornLocal.x * 1000.0f + 0.5f);
    h += cast(uint) cast(int) (bornLocal.y * 1000.0f + 0.5f) * 2654435761u;
    h += cast(uint) cast(int) (bornLocal.z * 1000.0f + 0.5f) * 40503u;
    h += cast(uint) sprouts * 2246822519u;
    h += cast(uint) round * 40503u;
    return mixIndex(h);
}

/// Сторона от средней линии: +1 справа, -1 слева. Ось «наружу от средней
/// линии» — это `side` по x, и она у зеркальных концов противоположна. Всё
/// остальное (вверх, назад) у них совпадает, поэтому одна и та же программа,
/// заданная в этом базисе, даёт на двух сторонах зеркальный результат — без
/// всякой проверки симметрии.
private float bodySide(const vec3 p, const float midX) pure nothrow @nogc
{
    return p.x >= midX ? 1.0f : -1.0f;
}

/// Направление пробы, повёрнутое в базис тела: локальное направление
/// разворачивается наружу от средней линии согласно стороне конца.
private vec3 toWorld(const vec3 local, const float side) pure nothrow @nogc
{
    return vec3(local.x * side, local.y, local.z);
}

/// Обратное к `toWorld`: направление мира в базисе тела.
private vec3 toLocal(const vec3 world, const float side) pure nothrow @nogc
{
    return vec3(world.x * side, world.y, world.z);
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

    /// Гены активной фазы роста — подменяются по ходу `develop`.
    SegmentGenes cur;

    /// Номер активной фазы.
    size_t phase;

    /// Фаза, в которой родился узел: по ней колесо берёт моторный ген.
    size_t[] nodePhase;

    Frame f;

    /// Узлы заданного каркаса кабины — ниже этого порога ничего не считается
    /// своим: колёса вешаются только на то, что выросло само.
    size_t scaffoldNodes;

    /// Балок, выпущено из каждого узла, — по этому счётчику узел замирает.
    size_t[] sprouts;

    /// Родитель каждого узла: у узлов заданного каркаса — `noParent`. По этой
    /// цепочке рост узнаёт свою ветвь.
    size_t[] parent;

    /// Направление балки, которой родился каждый узел: у точек подвески это
    /// «вниз», дальше — куда пошла предыдущая балка. Мерило того, где конец
    /// находится в своём сегменте, и единственное состояние, которое нужно
    /// ему для выбора направления.
    vec3[] born;

    /// Балок выросло всего.
    size_t grown;

    /// Сумма длин выросших балок — числитель `scale`.
    float grownLen = 0.0f;

    /// Характерный масштаб организма: средняя длина его балок. Запреты роста
    /// (расстояние между ветвями, слияние, виток) меряются им, а не базовым
    /// шагом: вытянувшийся организм иначе ветвит реже, чем велик запрет, и
    /// локальное торможение роста перестаёт его останавливать.
    float scale() const
    {
        return grown > 0 ? max(cur.stepLength, grownLen / grown) : cur.stepLength;
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
        const float score = tissueGain[tissue] * cur.actProduction * ai.act
            - cur.inhProduction * ai.inh + flowAt(p) + gravityAt(p)
            + midlineAt(p) - crowdShare * crowding(p, tip);
        return score / cur.threshold;
    }

    /// Отход от срединной плоскости: чем дальше от неё, тем сильнее ткань
    /// хочет расти дальше в ту же сторону.
    float midlineAt(const vec3 p) const
    {
        const float dx = abs(p.x - f.nodes[0].pos.x);
        return cur.midlineShare * (1.0f
            - expf(-(dx * dx) / (midlineDepth * midlineDepth)));
    }

    /// Длина новой балки: базовый шаг гена плюс вытяжка, которую даёт
    /// локальное поле. Поле богаче — балка длиннее, и так длина становится
    /// такой же эволюционируемой величиной, как и всё остальное.
    float budLength(const float resp) const
    {
        const float excess = max(0.0f, resp - respondGate);
        return min(cur.stepLength + stretchShare * cur.actDiffusion * excess,
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
            r += expf(-d / cur.motorDiffusion);
        }
        return r * cur.motorProduction;
    }

    Fields raw(const vec3 p) const
    {
        Fields r;
        foreach (n; f.nodes)
        {
            const float d = distance(n.pos, p);
            r.act += expf(-d / cur.actDiffusion);
            r.inh += expf(-d / cur.inhDiffusion);
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
        if (cur.flowStrength == 0.0f)
            return 0.0f;
        // Только вдоль курса: поперечная составляющая была антисимметричной
        // (пропорциональной `rel.x`) и ломала зеркальность поля. Отход в
        // стороны теперь задаёт organizer средней линии, он симметричен.
        return cur.flowStrength * flowCourseShare * alongCourse(p - f.nodes[0].pos);
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
        nodePhase ~= phase;
        const vec3 step = to - f.nodes[from].pos;
        born ~= step * (1.0f / sqrt(dot(step, step)));
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

/// Потокобезопасный буфер для почек
private class BudBuffer
{
    private Bud[] _data;
    private Mutex _m;

    this()
    {
        _m = new Mutex();
    }

    void append(Bud b)
    {
        synchronized (_m)
        {
            _data ~= b;
        }
    }

    Bud[] extract()
    {
        synchronized (_m)
        {
            return _data.dup;
        }
    }
}

/// Проба направления из конца `tip`: если место годится, почка попадает в
/// список. Габарит проверяется на базовом шаге, поле считаем после него —
/// отбраковка дешёвая.
private void addBud(const Growth g, size_t tip, const vec3 from, const vec3 d,
    BudBuffer buds)
{
    const vec3 probe = from + d * g.cur.stepLength;
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
    buds.append(Bud(tip, to, resp));
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
Nullable!Frame develop(const Chromosome chr)
{
    auto c = normalized(chr);
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
    // Зачаток — любой узел каркаса кабины: форма машины обязана быть
    // следствием генов, а не выбора «отсюда расти можно, а отсюда нет».
    g.sprouts = new size_t[g.scaffoldNodes];
    g.sprouts[] = 0;
    g.parent = new size_t[g.scaffoldNodes];
    g.parent[] = noParent;
    g.born = new vec3[g.scaffoldNodes];
    g.born[] = vec3(0.0f, 0.0f, -1.0f);
    g.nodePhase = new size_t[g.scaffoldNodes];

    // Фазы роста идут одна за другой: геном — список полей, и каждое
    // доращивает каркас по-своему. Бюджет фазы — прибавка к размеру, поэтому
    // поздние фазы строят поверх ранних.
    const size_t frameCap = maxBeamCount - cockpitFrameBeamCount();
    foreach (phase, seg; c.segments)
    {
        g.cur = seg;
        g.phase = phase;
        g.grownMax = min(g.grown + cast(size_t) seg.beamBudget, frameCap);
        foreach (round; 0 .. maxGrowthRounds)
            if (!growRound(g, round))
                break;
    }

    placeWheels(g);

    // Машина без колёс не едет: null — сигнал отбору, что хромосома мертва.
    if (g.f.anchors.length < 2)
        return Nullable!Frame.init;
    return Nullable!Frame(g.f);
}

/// Один такт роста: каждый конец сам выбирает, куда ему расти. `true`, если
/// что-то выросло, — значит, есть смысл сделать ещё такт.
///
/// Отбор локальный, по концам: конец решает за себя и ни с кем не соревнуется,
/// левый не делит место с правым. Общего списка почек и глобального отбора нет —
/// выбирать, кому достанется место, должно поле, а не код.
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

    const float midX = g.f.nodes[0].pos.x;

    // Узел считается лежащим на плоскости симметрии, если он ближе этого
    // зазора к срединной плоскости: там почка либо остаётся в плоскости, либо
    // отражается парой.
    const float planeEps = foldShare * g.scale();

    // Такт идёт в две фазы. Сначала каждый конец выбирает направление по полю,
    // каким оно было до такта, и решения не пишутся в каркас; затем выбранное
    // выращивается. Выбирать и растить вперемешку нельзя: конец, обработанный
    // раньше, меняет поле для соседнего, и тот отвечает уже на другом поле —
    // расхождение из-за порядка, а не из-за биологии.
    Bud[] chosen;
    foreach (tip; tips)
    {
        const vec3 from = g.f.nodes[tip].pos;
        const float side = bodySide(from, midX);
        // Серия углов задаётся состоянием самого конца: куда он смотрит в
        // своём сегменте, сколько отростков выпустил, какой такт. Координаты
        // на каркасе в ключ не входят — иначе инструкция переписывалась бы по
        // ходу роста, и каждый шаг зависел бы от всех предыдущих.
        const uint base = budSeed(round, g.sprouts[tip],
            toLocal(g.born[tip], side));
        auto budsBuf = new BudBuffer();
        const bool onPlane = abs(from.x - midX) <= planeEps;
        foreach (k; 0 .. directionProbes)
        {
            // Углы набираются в базисе тела и разворачиваются по стороне конца,
            // поэтому зеркальные концы обстреливаются зеркальными же углами.
            const uint s = base + cast(uint) k * 97u;
            addBud(g, tip, from, toWorld(spherePoint(s), side), budsBuf);
            if (onPlane)
            {
                // На самой плоскости узел пробует и строго срединные
                // направления: стойка руля растёт в плоскости и остаётся
                // собственным зеркалом, а вбок — уже зеркальной парой.
                const vec3 sp = spherePoint(s ^ 0x9e3779b9u);
                const vec3 inPlane = vec3(0.0f, sp.y, sp.z);
                const float len = sqrt(dot(inPlane, inPlane));
                if (len > 1e-3f)
                    addBud(g, tip, from, inPlane / len, budsBuf);
            }
        }
        auto buds = budsBuf.extract();
        if (buds.length != 0)
        {
            sort!((a2, b2) => a2.resp > b2.resp)(buds);
            chosen ~= buds[0];
        }
    }

    // Зеркальные почки растут парами: иначе float-шум на границе бюджета
    // ломает симметрию, хотя поле вокруг зеркальных концов одинаковое.
    struct Pair
    {
        Bud a, b;
        bool hasA, hasB;
        float bestResp;
    }
    Pair[] pairs;
    bool[] used = new bool[chosen.length];
    foreach (i, budA; chosen)
    {
        if (used[i])
            continue;
        const vec3 fromA = g.f.nodes[budA.tip].pos;
        const float sideA = bodySide(fromA, midX);
        // Узел на плоскости симметрии, у которого почка уходит вбок, растёт
        // зеркальной парой: поперечина руля возникает сама, без правила формы.
        // Срединная почка (стойка) остаётся одиночной, и узел — своим зеркалом.
        if (abs(fromA.x - midX) <= planeEps
            && abs(budA.to.x - midX) > planeEps)
        {
            used[i] = true;
            const vec3 toB = vec3(2.0f * midX - budA.to.x, budA.to.y,
                budA.to.z);
            pairs ~= Pair(budA, Bud(budA.tip, toB, budA.resp), true, true,
                budA.resp);
            continue;
        }
        const vec3 localA = toLocal(budA.to - fromA, sideA);
        bool paired;
        foreach (j, budB; chosen)
        {
            if (j <= i || used[j])
                continue;
            const vec3 fromB = g.f.nodes[budB.tip].pos;
            const float sideB = bodySide(fromB, midX);
            if (sideA * sideB >= 0.0f)
                continue;
            const vec3 localB = toLocal(budB.to - fromB, sideB);
            if (distance(localA, vec3(-localB.x, localB.y, localB.z)) < 1e-3f)
            {
                used[i] = true;
                used[j] = true;
                pairs ~= Pair(budA, budB, true, true,
                    max(budA.resp, budB.resp));
                paired = true;
                break;
            }
        }
        if (!paired)
        {
            used[i] = true;
            pairs ~= Pair(budA, Bud.init, true, false, budA.resp);
        }
    }
    sort!((x, y) => x.bestResp > y.bestResp)(pairs);
    foreach (p; pairs)
    {
        if (p.hasA && p.hasB)
        {
            if (g.grown + 2 > g.grownMax)
                continue;
            if (growBud(g, p.a) && growBud(g, p.b))
                grew = true;
        }
        else if (p.hasA)
        {
            if (g.grown >= g.grownMax)
                break;
            if (growBud(g, p.a))
                grew = true;
        }
    }
    return grew;
}

/// Врастить почечную балку. `false` — место негодное: выжжено полем, занято
/// чужим узлом или слишком близко к своему предку.
private bool growBud(ref Growth g, const Bud bud)
{
    // Поле пересчитывается на каждой выросшей балке: только что выросший узел
    // сам стал источником и может задушить соседние места.
    if (g.response(bud.to, frameTissue) <= respondGate)
        return false;
    // Вытяжка по полю не должна перескакивать выжженный участок: балка лежит
    // только там, где ткань жива по всей длине.
    if (!g.aliveAlong(g.f.nodes[bud.tip].pos, bud.to))
        return false;
    const auto merged = g.mergeTarget(bud.to, bud.tip);
    if (merged >= 0)
    {
        // Балка к узлу-слиянию уже есть — расти тут больше нечего, а новый
        // узел в чужой точке поставил бы дубль и вторую копию той же балки.
        if (g.linked(bud.tip, cast(size_t) merged))
            return false;
        g.f.beams ~= new Beam(bud.tip, cast(size_t) merged, g.c.beamRadius);
        g.sprouts[bud.tip] += 1;
        g.grown += 1;
        g.grownLen += distance(g.f.nodes[bud.tip].pos,
            g.f.nodes[cast(size_t) merged].pos);
        return true;
    }
    if (g.tooClose(bud.to, bud.tip))
        return false;
    g.addBeam(bud.tip, bud.to);
    growWheelAxle(g, g.f.nodes.length - 1);
    return true;
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
        // Колесо слышит поле той фазы, в которой вырос его узел: корпус несёт
        // моторные гены, вилка — нет, и её колёса выходят пассивными сами.
        g.cur = g.c.segments[g.nodePhase[i]];
        const vec3 p = g.f.nodes[i].pos;
        // Ось колеса поперечная, и подвеска должна заходить по ней: балка
        // поперёк идёт по оси, внутри ступицы, и резину не трогает. Отсюда
        // бонус концу, чья несущая балка поперечна.
        const float wheel = g.response(p, wheelTissue)
            + wheelAxleShare * axleShare(g, i);
        if (!g.isTip(i))
            continue;
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

/// Поперечная ось под колесо: конец, который слышит ткань колеса, но несёт
/// нога под углом, отращивает короткую ось вдоль ступицы. Тогда покрышку
/// режет только сама нога, а она кончается до резины.
private void growWheelAxle(ref Growth g, size_t tip)
{
    const vec3 p = g.f.nodes[tip].pos;
    if (g.rawMotor(p) <= motorGate)
        return;
    if (g.response(p, wheelTissue) <= respondGate)
        return;
    if (axleShare(g, tip) > 0.7f)
        return;

    const float scale = g.c.wheelRadius / defaultWheelRadius;
    const float len = 0.5f * wheelWidth * scale + g.c.beamRadius + wheelGap;
    const vec3 hub = p + wheelAxle(forward, up, p) * len;
    if (abs(hub.x - g.f.nodes[0].pos.x) > maxHalfWidth)
        return;
    g.addBeam(tip, hub);
}

/// Влезает ли колесо на этот конец.
///
/// Три ограничения, и все они — геометрия, а не договорённость:
///   - обод не наезжает на уже поставленные колёса;
///   - ни одна балка не задевает покрышку, своя несущая included: ось через
///     ступицу полосу не режет, а наклонная балка режет резину;
///   - ось колеса выше земли хотя бы на половину радиуса, иначе колесо сразу
///     уходит под грунт.
private bool wheelFits(const Growth g, size_t node)
{
    const vec3 p = g.f.nodes[node].pos;
    const float r = g.c.wheelRadius;

    foreach (a; g.f.anchors)
        if (distance(p, g.f.nodes[a.node].pos) < r + a.radius + wheelGap)
            return false;

    if (beamHitsTyre(g.f, node, g.c.wheelRadius, g.c.beamRadius))
        return false;
    return true;
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

unittest
{
    import frame.frame : isConnected;
    import genetics.fitness : buggyFitness;
    import physics_world.physics : steerNodeIndex;

    static size_t degree(ref const Frame fr, size_t n)
    {
        size_t d = 0;
        foreach (b; fr.beams)
            if (b !is null && (b.a == n || b.b == n))
                ++d;
        return d;
    }

    // Основатель вида обязан вырасти в машину, а не в комок: каркас связный,
    // есть колёса и хотя бы одно ведущее.
    const Chromosome founder;
    const auto grown = develop(founder);
    assert(!grown.isNull, "основатель обязан вырасти в машину с колёсами");

    const auto f = grown.get;
    assert(f.beams.length > cockpitFrameBeamCount(), "основатель обязан что-то дорастить");
    assert(isConnected(f), "рост не оставляет висячих узлов");
    assert(f.anchors.length >= 2, "машине нужно хотя бы два колеса");
    size_t motor, passive;
    foreach (a; f.anchors)
        if (a.kind == AnchorKind.motorWheel)
            ++motor;
        else
            ++passive;
    assert(motor >= 1, "без ведущего колеса машина не едет");
    assert(passive == 2, "ровно два пассивных колеса на концах вилки");
    assert(f.motorPower == founder.motorPower, "сила мотора — из гена");
    assert(buggyFitness(f) > 0.0f, "основатель обязан быть жизнеспособен");

    // Симметрия по X: каждый узел имеет зеркальный относительно срединной
    // плоскости каркаса.
    foreach (n; f.nodes)
    {
        bool mirror;
        foreach (m; f.nodes)
            if (abs(m.pos.x + n.pos.x) < 1e-4f && abs(m.pos.y - n.pos.y) < 1e-4f
                && abs(m.pos.z - n.pos.z) < 1e-4f)
            {
                mirror = true;
                break;
            }
        assert(mirror, "каркас не симметричен по X");
    }

    // Вилка: стойка от узла руля спускается к ступице степени 3, из которой
    // растут два конца степени 1 с пассивными колёсами.
    size_t hub = size_t.max;
    foreach (b; f.beams)
    {
        if (b is null || (b.a != steerNodeIndex && b.b != steerNodeIndex))
            continue;
        const h = (b.a == steerNodeIndex) ? b.b : b.a;
        if (f.nodes[h].pos.z < f.nodes[steerNodeIndex].pos.z
            && dot(f.nodes[h].pos - f.nodes[steerNodeIndex].pos, forward) > 0.0f)
            hub = h;
    }
    assert(hub != size_t.max, "стойка руля должна спускаться вперёд вниз");
    assert(degree(f, hub) == 3, "ступица вилки должна иметь степень 3");
    size_t[2] ends;
    size_t nEnds;
    foreach (b; f.beams)
    {
        if (b is null || (b.a != hub && b.b != hub))
            continue;
        const e = (b.a == hub) ? b.b : b.a;
        if (e == steerNodeIndex)
            continue;
        assert(nEnds < 2, "у ступицы не должно быть лишних концов");
        ends[nEnds] = e;
        ++nEnds;
    }
    assert(nEnds == 2, "из ступицы должно расти ровно два конца степени 1");
    foreach (end; ends)
        assert(degree(f, end) == 1, "конец поперечины должен быть листом");
    bool[2] wheeled;
    foreach (e, end; ends)
        foreach (a; f.anchors)
            if (a.node == end)
                wheeled[e] = a.kind == AnchorKind.wheel;
    assert(wheeled[0] && wheeled[1],
        "концы поперечины должны нести пассивные колёса");
}
