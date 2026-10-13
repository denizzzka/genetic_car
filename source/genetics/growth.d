module genetics.growth;

import std.algorithm : canFind, max, min, sort;
import std.math : PI, abs, cos, sin, sqrt;
import std.typecons : Nullable;

// `std.math.exp` для float считается портабельным полиномом — в цикле поля
// это втрое дороже libc-варианта.
import core.stdc.math : expf;

import dlib.math.vector : distance, dot, vec3;
import dlib.math.utils : clamp;
import core.sync.mutex;

debug import std.stdio : writeln;

import frame.frame : Anchor, AnchorKind, Beam, Frame, Node, forward, up,
    alongCourse, right;
import physics_world.physics : wheelInnerRadius, wheelWidth;
import physics_world.wheel : defaultWheelRadius, isOwnWheel, wheelAxle;

import frame.cockpit : beamHitsCabin, cockpitFrameContext,
    cockpitFrameBeamCount, cockpitFrameNodeCount, cockpitGeometry,
    cockpitMountNodes, wheelHitsCabin;
import genetics.fitness : maxBeamCount;
import genetics.chromosome : Chromosome, SegmentGenes, normalized;

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

    debug string parts(const vec3 p, size_t tip) const
    {
        import std.format : format;
        const auto ai = raw(p);
        return format("act=%.3f inh=%.3f flow=%.3f grav=%.3f mid=%.3f "
            ~ "crowd=%.3f",
            tissueGain[frameTissue] * cur.actProduction * ai.act,
            -cur.inhProduction * ai.inh, flowAt(p), gravityAt(p), midlineAt(p),
            -crowdShare * crowding(p, tip));
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
        debug writeln("PHASE ", phase, " done grown=", g.grown, "/",
            g.grownMax, " beams=", g.f.beams.length);
    }

    debug foreach (i, b; g.f.beams)
        if (i >= cockpitFrameBeamCount())
            writeln("GBEAM ", b.a, "-", b.b, " phase=",
                g.nodePhase[b.a > b.b ? b.a : b.b],
                " len=", distance(g.f.nodes[b.a].pos, g.f.nodes[b.b].pos));

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
    debug foreach (b; chosen)
        if (b.tip == 1 || b.tip == 12 || b.tip == 10 || b.tip == 6)
            writeln("CHOSEN ph=", g.phase, " rd=", round, " tip=", b.tip,
                " resp=", b.resp, " to=(", b.to.x, ",", b.to.y, ",", b.to.z,
                ") ", g.parts(b.to, b.tip));
    debug foreach (p; pairs)
        writeln("PAIR ph=", g.phase, " rd=", round,
            " A=", p.hasA ? p.a.tip : size_t.max,
            " B=", p.hasB ? p.b.tip : size_t.max,
            " resp=", p.bestResp, " grown=", g.grown, "/", g.grownMax);
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
        debug writeln("WSPOT node=", i, " phase=", g.nodePhase[i], " tip=",
            g.isTip(i), " chain=", g.onSingleChain(i), " wheel=", wheel,
            " motor=", g.rawMotor(p), " cabin=",
            wheelHitsCabin(g.f, p, g.c.wheelRadius));
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

    if (beamHitsTyre(g, node))
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

/// Задевает ли отрезок полосу покрышки: радиальный профиль `(радиус, осевая
/// координата)` пересекается с полосой [inner, outer] по радиусу и
/// [-halfWidth, halfWidth] по оси. Обе величины линейны вдоль отрезка, поэтому
/// пересечение полосы — это пересечение двух интервалов параметра.
///
/// Ступица — начало координат: отрезок, пришедший в неё, имеет нулевой
/// радиус в самой ступице. Настоящая резина начинается на `inner`, поэтому
/// глубина проникновения считается от той точки, где отрезок входит в
/// полосу, и балка, кончающаяся в ступице, резины не режет.
private bool segmentHitsTyreBand(const vec3 pa, const vec3 pb,
    const float inner, const float outer, const float halfWidth)
{
    // Отрезок, целиком лежащий на оси (обе точки при нулевом радиусе),
    // полосу не режет: своя несущая балка проходит сквозь ступицу.
    if (pa.y <= inner && pb.y <= inner)
        return false;
    float lo = 0.0f, hi = 1.0f;
    if (!clipBand(pa.x, pb.x, -halfWidth, halfWidth, lo, hi))
        return false;
    if (!clipBand(pa.y, pb.y, inner, outer, lo, hi))
        return false;
    // Радиус на входе в полосу: у отрезка, пришедшего в ступицу, он равен
    // внутреннему радиусу покрышки — это и есть своя несущая балка.
    const float radAt = pa.y + (pb.y - pa.y) * lo;
    return radAt > inner;
}

/// Сузить интервал параметра до части отрезка, лежащей внутри полосы
/// [loV, hiV]. Отрезок, пересекающий полосу, но не удержавшийся в интервале,
/// отвергается.
private bool clipBand(const float a, const float b, const float loV, const float hiV,
    ref float lo, ref float hi)
{
    const float d = b - a;
    if (abs(d) < 1e-7f)
        return a >= loV && a <= hiV;
    float t0 = (loV - a) / d, t1 = (hiV - a) / d;
    if (t0 > t1)
    {
        const float t = t0;
        t0 = t1;
        t1 = t;
    }
    lo = max(lo, t0);
    hi = min(hi, t1);
    return lo <= hi;
}

/// Задевает ли балка покрышку колеса на этом конце. Проверка идёт в системе
/// колеса: ось диска поперечна курсу, и только тогда радиус от оси — это
/// расстояние до резины. Балка, идущая по оси (в том числе своя несущая,
/// кончающаяся в ступице), полосу не режет; наклонная — режет.
private bool beamHitsTyre(const Growth g, size_t node)
{
    const vec3 hub = g.f.nodes[node].pos;
    const float scale = g.c.wheelRadius / defaultWheelRadius;
    const float inner = wheelInnerRadius * scale - g.c.beamRadius;
    const float outer = g.c.wheelRadius + g.c.beamRadius;
    const float halfWidth = 0.5f * wheelWidth * scale + g.c.beamRadius;
    const vec3 axle = wheelAxle(forward, up, hub);

    foreach (b; g.f.beams)
    {
        // Свою ступицу балка не режет: ось законно проходит через собственное
        // колесо. Полосу покрышки балка пересекает только когда подходит к
        // ступице под углом, а не вдоль оси.
        if (isOwnWheel(node, b.a, b.b))
            continue;
        const vec3 pa = projectTyre(g.f.nodes[b.a].pos - hub, axle);
        const vec3 pb = projectTyre(g.f.nodes[b.b].pos - hub, axle);
        if (segmentHitsTyreBand(pa, pb, inner, outer, halfWidth))
            return true;
    }
    return false;
}

/// Перевести точку в систему колеса: поперёк оси — `x`, радиальное
/// расстояние от оси раскладывается в `y` (радиус) и `z` (нуль, чтобы
/// `segmentHitsTyreBand` считала ровно один радиус, а не два).
private vec3 projectTyre(const vec3 p, const vec3 axle)
{
    const float axial = dot(p, axle);
    const vec3 radial = p - axle * axial;
    return vec3(axial, sqrt(dot(radial, radial)), 0.0f);
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

// Временный перебор генов фазы поперечины: ищем вилку от steerNodeIndex.
debug void growthSweep()
{
    import genetics.chromosome : Chromosome, SegmentGenes;
    import std.stdio : writefln;
    import physics_world.physics : steerNodeIndex;

    static size_t deg(ref const Frame fr, size_t n)
    {
        size_t d = 0;
        foreach (bb; fr.beams)
        {
            const b = cast(Beam) bb;
            if (b !is null && (b.a == n || b.b == n))
                ++d;
        }
        return d;
    }

    const SegmentGenes stem = SegmentGenes(1.2f, 0.10f, 0.32f, 1.20f, 0.42f,
        2.00f, 0.0f, 0.08f, 1.0f, 0.0f, 2.5f);
    const SegmentGenes body = SegmentGenes(1.2f, 0.10f, 0.12f, 0.30f, 0.42f,
        1.00f, 20.0f, 0.10f, 4.0f, 0.2f, 2.5f);

    foreach (ad; [0.10f, 0.20f, 0.32f])
    foreach (step; [0.20f, 0.30f, 0.40f])
    foreach (flow; [0.20f, 0.30f, 0.40f])
    {
        Chromosome c;
        c.segments = [
            stem,
            body,
            SegmentGenes(0.20f, 0.10f, ad, 1.20f, 0.10f, flow, 2.0f, step,
                2.0f, 0.0f, 2.5f),
        ];
        const auto grown = develop(c);
        if (grown.isNull)
            continue;
        const auto f = grown.get;
        bool ok = false;
        size_t hub;
        size_t[2] forkEnds;
        float d01;
        float whiskLen;
        foreach (i, bb; f.beams)
        {
            const b = cast(Beam) bb;
            if (b is null)
                continue;
            if (b.a != steerNodeIndex && b.b != steerNodeIndex)
                continue;
            const h = (b.a == steerNodeIndex) ? b.b : b.a;
            const vec3 toHub = f.nodes[h].pos - f.nodes[steerNodeIndex].pos;
            if (dot(toHub, forward) <= 0.0f)
                continue;
            size_t[2] halves, ends;
            size_t nh = 0;
            foreach (j, b2b; f.beams)
            {
                if (j == i)
                    continue;
                const b2 = cast(Beam) b2b;
                if (b2 is null || (b2.a != h && b2.b != h))
                    continue;
                if (nh == 2) { nh = 3; break; }
                halves[nh] = j;
                ends[nh] = (b2.a == h) ? b2.b : b2.a;
                ++nh;
            }
            if (nh != 2 || deg(f, h) != 3)
                continue;
            const vec3 st = toHub / toHub.length;
            const vec3 e0 = (f.nodes[ends[0]].pos - f.nodes[h].pos)
                / (f.nodes[ends[0]].pos - f.nodes[h].pos).length;
            const vec3 e1 = (f.nodes[ends[1]].pos - f.nodes[h].pos)
                / (f.nodes[ends[1]].pos - f.nodes[h].pos).length;
            if (dot(e0, st) < 0.5f && dot(e1, st) < 0.5f && dot(e0, e1) < 0.0f
                && deg(f, ends[0]) == 1 && deg(f, ends[1]) == 1)
            {
                ok = true;
                hub = h;
                d01 = dot(e0, e1);
                forkEnds = ends;
                whiskLen = distance(f.nodes[h].pos, f.nodes[ends[0]].pos);
                break;
            }
        }
        size_t pas = 0, mot = 0;
        foreach (a; f.anchors)
            if (a.kind == AnchorKind.motorWheel)
                ++mot;
            else
                ++pas;
        if (ok)
        {
            const bool e0w = f.anchors.canFind!((a) => a.node == forkEnds[0]);
            const bool e1w = f.anchors.canFind!((a) => a.node == forkEnds[1]);
            string ek;
            foreach (a; f.anchors)
                if (a.node == forkEnds[0] || a.node == forkEnds[1])
                    ek ~= (a.kind == AnchorKind.motorWheel ? "M" : "w");
            writefln("FORK ad=%.2f step=%.2f flow=%.2f "
                ~ "hubZ=%.2f dot01=%.2f whisk=%.2f e0=(%.2f,%.2f) pas=%d mot=%d "
                ~ "wheeled=%d,%d %s",
                ad, step, flow, f.nodes[hub].pos.z, d01, whiskLen,
                f.nodes[forkEnds[0]].pos.y, f.nodes[forkEnds[0]].pos.z,
                pas, mot, e0w, e1w, ek);
        }
    }
}
