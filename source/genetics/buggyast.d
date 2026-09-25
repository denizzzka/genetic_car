module genetics.buggyast;

import std.math;
import std.typecons: Nullable;
import dlib.math.vector;
import frame.frame;
import physics_world.wheel : defaultWheelRadius;
import genetics.sge;

/**
 * Терминалы грамматики багги — «команды развития», которые `decode` выводит
 * из генома. `buildAst` собирает из них дерево, `frameFromAst` выполняет
 * в порядке следования, строя каркас от первого узла. Часть токенов несёт
 * значение (float/int в `payload`), часть — пустые маркеры.
 */
enum Tok
{
    /// Маркер начала сегмента: ограничивает группу балок, строящихся как
    /// одно модульное целое. Каждый сегмент сам решает, раздваиваться ли.
    segStart,

    /// Маркер раздвоения сегмента: если стоит после `segStart`, каждая балка
    /// сегмента рождает пару (twin) относительно оси сегмента — X стартового
    /// узла сегмента (не мировой X == 0). Ось дают локальные пары конечностей;
    /// пустой маркер — медианная одиночная структура («глаз по центру»).
    fork,

    /// Nodal — локальный активатор асимметрии twin-пары (float). Знак задаёт,
    /// в какую сторону уводятся twin-балки, величина — силу сдвига. Складывается
    /// с организменным градиентом `lrGradient`, а подавляется локальным
    /// ингибитором `lefty` — как в LR-генезе позвоночных. Одиночные
    /// медианные балки (без twin) активатор не затрагивает совсем.
    nodal,

    /// Lefty — локальный ингибитор асимметрии (float, ≥ 0). Нелинейно гасит
    /// суммарный активатор: чем сильнее `nodal + lrGradient`, тем больше
    /// подавление (`lefty·act²`) — так асимметрия пары остаётся малой и
    /// самоограниченной, а не «разносит» структуру. Ноль — активатор
    /// действует в полную силу.
    lefty,

    /// Множитель порога билатеральности (float, ≥ 0): эффективная асимметрия
    /// ниже `bilateralThreshold · tok` не материализуется, twin-пара строится
    /// строго зеркально. Эволюционируемый параметр развития: сильный порог
    /// «замораживает» билатеральность, нулевой открывает асимметрию полностью.
    bilateralThreshold,

    /// Начало балки: ссылка на последний созданный узел (маркер без значения).
    refLast,

    /// Начало балки: ссылка на базовый узел сегмента (маркер без значения).
    /// «Зачаток»: узел, на котором начался сегмент. Возвращает рост к нему,
    /// так из одной точки ветвления можно выпускать несколько отростков
    /// (пальцы из «запястья»), не кодируя точный номер узла.
    refBase,

    /// Ссылка на существующий узел по номеру (int): начало/конец балки
    /// или индекс узла якоря. В якорях заворачивается по числу узлов.
    refIdx,

    /// Маркер конца балки: создать новый узел на позиции `старта + смещение`
    /// (после маркера идут три токена `coord`).
    endNew,

    /// Маркер конца балки: как `endNew`, но растущий конец сливается
    /// с ближайшим существующим узлом (кроме старта) в пределах
    /// `mergeRadius`; иначе — как `endNew`.
    endNear,

    /// Координата (float): абсолютная позиция первого узла (seed)
    /// в осях `startForward/startRight/startUp` или дельта относительно
    /// старта балки в осях `forward/right/up` (вперёд/вправо/вверх).
    coord,

    /// Радиус трубы балки (float, базовый семпл 0.02..0.06),
    /// до интерпретации масштабируется морфоген-градиентом.
    radius,

    /// Тип балки (int-код `BeamKind`).
    beamKind,

    /// Маркер: конец списка балок, начало списка якорей (колёс).
    anchors,

    /// Тип якоря-колеса (int-код `AnchorKind`).
    anchorKind,

    /// Радиус колеса якоря (float). Каждый якорь носит свой радиус.
    wheelRadius,

    /// Морфоген: во сколько раз сужается толщина к последней балке
    /// (float, 0.4..1.0). Токен изымается из потока до интерпретации.
    taper,

    /// Морфоген: показатель степени кривой градиента толщины
    /// (float, 0.5..4.0). Токен изымается из потока до интерпретации.
    taperPow,

    /// Turtle: начальный заголовок построения (float, радианы). Дельты
    /// `endNew`/`endNear` интерпретируются в системе заголовка: значение
    /// семпла `forward` — вперёд по заголовку, `right` — вправо от него,
    /// `up` — вертикально вверх.
    heading,

    /// Turtle: приращение заголовка после балки (float, радианы).
    /// Накапливается в общее направление построения.
    turn,

    /// Сила мотор-колёс (float, Н·м) — наследуемый параметр развития.
    /// Задаёт момент, развиваемый ведущими колёсами; эволюция подбирает его
    /// под геометрию, чтобы машина ехала, а не опрокидывалась.
    motorPower,

    /// Организменный LR-морфоген (float): знак задаёт полярность «лево/право»
    /// всего тела, величина — силу градиента. Наследуется всеми сегментами —
    /// сомиты сегментируют единый план строения, поэтому общий знак есть у
    /// организма, а не у каждой балки. Входит в активатор вместе с `nodal`.
    lrGradient,
}

enum StartRefKind { last, base, idx }
struct StartRef
{
    StartRefKind kind;
    size_t idx;
}

enum EndRefKind { newNode, nearNode, idx }
struct EndRef
{
    EndRefKind kind;
    vec3 delta;
    size_t idx;
}

struct BeamAst
{
    StartRef start;
    EndRef end;
    float radius;
    float nodal;
    float lefty;

    /// Локальный множитель порога билатеральности: ноль — асимметрия не
    /// заблокирована (необязательный токен, ручные потоки его опускают).
    float threshold = 0.0f;
    BeamKind kind;
    float turn;
}

struct SegmentAst
{
    bool fork;
    BeamAst[] beams;
}

struct AnchorAst
{
    AnchorKind kind;
    size_t idx;
    float radius = defaultWheelRadius;
}

struct Ast
{
    vec3 seed;
    float heading;
    float taper = 1.0f;
    float taperPow = 1.0f;
    float motorPower = 0.0f;

    /// Организменный LR-градиент: полярность лево/право всего тела.
    /// Знак — куда смещён «лево»-полюс, величина — сила градиента.
    float lrGradient = 0.0f;
    SegmentAst[] segments;
    AnchorAst[] anchors;
}

/**
 * Построить AST развития из потока терминалов.
 *
 * Дерево хранит все параметры грамматики без геометрии: startPos (seed,
 * heading, морфоген taper/taperPow), сегменты (fork, балки), балки
 * (старт/конец как типизированные рефы, радиус, активатор-ингибитор
 * nodal/lefty, turn) и якоря.
 * Геометрия здесь не строится — это этап разбора, а не интерпретации.
 */
Nullable!Ast buildAst(const Terminal!Tok[] tokens)
{
    Ast ast;
    size_t i = 0;

    if (i + 2 >= tokens.length)
        return Nullable!Ast.init;
    if (tokens[i].tok != Tok.coord || tokens[i + 1].tok != Tok.coord || tokens[i + 2].tok != Tok.coord)
        return Nullable!Ast.init;
    ast.seed = vec3(tokens[i].f, tokens[i + 1].f, tokens[i + 2].f);
    i += 3;

    // Морфоген-градиент толщины — параметры каркаса, читаются из startPos.
    if (i < tokens.length && tokens[i].tok == Tok.taper)
    {
        ast.taper = tokens[i].f;
        ++i;
    }
    if (i < tokens.length && tokens[i].tok == Tok.taperPow)
    {
        ast.taperPow = tokens[i].f;
        ++i;
    }

    if (tokens.length <= i || tokens[i].tok != Tok.heading)
        return Nullable!Ast.init;
    ast.heading = tokens[i].f;
    ++i;

    // Сила мотор-колёс — необязательный параметр (ручные потоки его опускают).
    if (i < tokens.length && tokens[i].tok == Tok.motorPower)
    {
        ast.motorPower = tokens[i].f;
        ++i;
    }

    // Организменный LR-градиент — необязательный: вручную собранные потоки
    // описывают тело без полярности, и тогда организм строго двусторонний.
    if (i < tokens.length && tokens[i].tok == Tok.lrGradient)
    {
        ast.lrGradient = tokens[i].f;
        ++i;
    }

    while (i < tokens.length && tokens[i].tok != Tok.anchors)
    {
        if (tokens[i].tok != Tok.segStart)
            return Nullable!Ast.init;
        ++i;

        SegmentAst seg;
        if (i < tokens.length && tokens[i].tok == Tok.fork)
        {
            seg.fork = true;
            ++i;
        }

        while (i < tokens.length && tokens[i].tok != Tok.segStart
            && tokens[i].tok != Tok.anchors)
        {
            BeamAst b;
            switch (tokens[i].tok)
            {
                case Tok.refLast:
                    b.start.kind = StartRefKind.last;
                    ++i;
                    break;
                case Tok.refBase:
                    b.start.kind = StartRefKind.base;
                    ++i;
                    break;
                case Tok.refIdx:
                    b.start.kind = StartRefKind.idx;
                    b.start.idx = tokens[i].i;
                    ++i;
                    break;
                default:
                    return Nullable!Ast.init;
            }

            switch (tokens[i].tok)
            {
                case Tok.endNew:
                    if (i + 3 >= tokens.length)
                        return Nullable!Ast.init;
                    if (tokens[i + 1].tok != Tok.coord
                        || tokens[i + 2].tok != Tok.coord
                        || tokens[i + 3].tok != Tok.coord)
                        return Nullable!Ast.init;
                    b.end.kind = EndRefKind.newNode;
                    b.end.delta = vec3(tokens[i + 1].f, tokens[i + 2].f, tokens[i + 3].f);
                    i += 4;
                    break;
                case Tok.endNear:
                    if (i + 3 >= tokens.length)
                        return Nullable!Ast.init;
                    if (tokens[i + 1].tok != Tok.coord
                        || tokens[i + 2].tok != Tok.coord
                        || tokens[i + 3].tok != Tok.coord)
                        return Nullable!Ast.init;
                    b.end.kind = EndRefKind.nearNode;
                    b.end.delta = vec3(tokens[i + 1].f, tokens[i + 2].f, tokens[i + 3].f);
                    i += 4;
                    break;
                case Tok.refIdx:
                    b.end.kind = EndRefKind.idx;
                    b.end.idx = tokens[i].i;
                    ++i;
                    break;
                default:
                    return Nullable!Ast.init;
            }

            if (tokens[i].tok != Tok.radius)
                return Nullable!Ast.init;
            b.radius = tokens[i].f;
            ++i;

            if (tokens[i].tok != Tok.nodal)
                return Nullable!Ast.init;
            b.nodal = tokens[i].f;
            ++i;

            if (tokens[i].tok != Tok.lefty)
                return Nullable!Ast.init;
            b.lefty = tokens[i].f;
            ++i;

            // Порог билатеральности — необязательный токен (ручные потоки его
            // опускают, тогда мёртвая зона не блокирует асимметрию вовсе).
            if (i < tokens.length && tokens[i].tok == Tok.bilateralThreshold)
            {
                b.threshold = tokens[i].f;
                ++i;
            }

            if (tokens[i].tok != Tok.beamKind)
                return Nullable!Ast.init;
            b.kind = cast(BeamKind) tokens[i].i;
            ++i;

            if (tokens[i].tok != Tok.turn)
                return Nullable!Ast.init;
            b.turn = tokens[i].f;
            ++i;

            seg.beams ~= b;
        }
        ast.segments ~= seg;
    }

    if (i >= tokens.length)
        return Nullable!Ast(ast);

    // Якоря: пара `anchorKind` + `refIdx` (индекс узла, заворачивается по
    // числу узлов в интерпретаторе — синтаксис тут ничего не решает).
    ++i;
    while (i < tokens.length)
    {
        if (tokens[i].tok != Tok.anchorKind)
            return Nullable!Ast.init;
        auto kind = cast(AnchorKind) tokens[i].i;
        ++i;

        if (i >= tokens.length || tokens[i].tok != Tok.refIdx)
            return Nullable!Ast.init;
        AnchorAst a;
        a.kind = kind;
        a.idx = tokens[i].i;
        ++i;

        // Радиус колеса — необязательный токен (ручные потоки его опускают,
        // тогда берётся defaultWheelRadius).
        if (i < tokens.length && tokens[i].tok == Tok.wheelRadius)
        {
            a.radius = tokens[i].f;
            ++i;
        }

        ast.anchors ~= a;
    }
    return Nullable!Ast(ast);
}

/**
 * Видовая мёртвая зона морфогенеза: асимметрия слабее этого порога не
 * материализуется в геометрию — шум развития её всё равно съел бы.
 * Это НЕ ген, а свойство самого механизма развития (как константа
 * диссоциации рецептора, одинаковая у всех организмов вида); эволюция
 * управляет им множителем `Tok.bilateralThreshold`.
 */
enum float bilateralThreshold = 0.02f;

/**
 * Асимметрия с учётом организменного градиента и локального ингибитора.
 *
 * Ответная кривая активатор-ингибитор: локальный `nodal` складывается с
 * организменным `lrGradient` в единый активатор, а `lefty` гасит именно
 * сумму квадратично (`lefty·act²`) — при сильном активаторе подавление
 * растёт, и асимметрия остаётся малой и самограниченной, как в LR-генезе
 * позвоночных. Общий знак градиента даёт телу полярность «лево/право»,
 * которой нет у отдельной балки. Для медианных одиночных балок (без twin)
 * асимметрия не применяется вовсе.
 */
float organismAsymmetry(float lrGradient, float nodal, float lefty)
{
    const activator = nodal + lrGradient;
    return activator / (1.0f + lefty * activator * activator);
}

/**
 * Асимметрия, реально материализуемая в twin-паре: отклик
 * активатор-ингибитор, срезанный порогом билатеральности. Пока отклик
 * слабее порога, пара строится строго зеркально.
 */
float beamAsymmetry(float lrGradient, const BeamAst b)
{
    const float eff = organismAsymmetry(lrGradient, b.nodal, b.lefty);
    return abs(eff) < bilateralThreshold * b.threshold ? 0.0f : eff;
}

unittest
{
    // Nodal/Lefty: слабый активатор почти симметричен, сильный гасится.
    assert(abs(organismAsymmetry(0.0f, 0.0f, 0.0f)) < 1e-7f,
        "нулевой nodal — точная симметрия");
    const strong = organismAsymmetry(0.0f, 0.3f, 0.0f);
    assert(strong > 0.2f, "без ингибитора активатор действует в полную силу");
    const damped = organismAsymmetry(0.0f, 0.3f, 1.0f);
    assert(damped > 0.0f && damped < strong,
        "ингибитор гасит активатор нелинейно");
    assert(organismAsymmetry(0.0f, -0.1f, 0.0f) < 0.0f,
        "знак активатора переворачивает сдвиг");
}

unittest
{
    // Организменный градиент: полярность тела — знак, а локальный nodal
    // складывается с ним в общий активатор, который гасит lefty.
    assert(abs(organismAsymmetry(0.1f, 0.0f, 0.0f)
            - organismAsymmetry(0.0f, 0.1f, 0.0f)) < 1e-7f,
        "организменный градиент без локального nodal — тот же активатор");
    assert(organismAsymmetry(-0.1f, 0.0f, 0.0f) < 0.0f
        && organismAsymmetry(0.1f, 0.0f, 0.0f) > 0.0f,
        "организменный градиент задаёт полярность лево/право");
    assert(abs(organismAsymmetry(0.2f, 0.0f, 1.0f)
            - organismAsymmetry(0.0f, 0.2f, 1.0f)) < 1e-7f,
        "ингибитор гасит суммарный активатор, а не локальную часть");
}

unittest
{
    // Порог билатеральности: отклик слабее порога не материализуется —
    // пара строго зеркальна; выше порога сдвиг направленный.
    BeamAst b;
    b.nodal = 0.01f;
    b.lefty = 0.0f;
    b.threshold = 0.0f;
    assert(beamAsymmetry(0.0f, b) > 0.0f,
        "нулевой порог не блокирует асимметрию вовсе");

    b.threshold = 1.0f;
    assert(beamAsymmetry(0.0f, b) == 0.0f,
        "отклик слабее видовой мёртвой зоны не материализуется");

    b.threshold = organismAsymmetry(0.0f, b.nodal, b.lefty)
        / (2.0f * bilateralThreshold);
    assert(beamAsymmetry(0.0f, b) > 0.0f,
        "порог ниже отклика пропускает направленный сдвиг");

    // Организменный градиент проходит тот же порог: он тоже часть активатора.
    b.threshold = 1.0f;
    b.nodal = 0.0f;
    b.lefty = 0.0f;
    assert(beamAsymmetry(0.01f, b) == 0.0f,
        "слабый организменный градиент тоже гасится мёртвой зоной");
    assert(beamAsymmetry(0.2f, b) > 0.0f,
        "сильный организменный градиент пробивает порог");
}

unittest
{
    // AST: структура грамматики видна без геометрии — сегмент, рефы, якоря.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.5f);
    t ~= new Terminal!Tok(Tok.coord, -0.25f);
    t ~= new Terminal!Tok(Tok.coord, 0.1f);
    t ~= new Terminal!Tok(Tok.taper, 0.6f);
    t ~= new Terminal!Tok(Tok.taperPow, 2.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.3f);
    t ~= new Terminal!Tok(Tok.motorPower, 77.0f);
    t ~= new Terminal!Tok(Tok.lrGradient, -0.12f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refBase);
    t ~= new Terminal!Tok(Tok.endNear);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.05f);
    t ~= new Terminal!Tok(Tok.nodal, 0.05f);
    t ~= new Terminal!Tok(Tok.lefty, 0.6f);
    t ~= new Terminal!Tok(Tok.bilateralThreshold, 1.5f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.7f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.motorWheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) 3);
    t ~= new Terminal!Tok(Tok.wheelRadius, 0.28f);

    auto ast = buildAst(t);
    assert(!ast.isNull);
    assert(abs(ast.get.seed.x - 0.5f) < 1e-6f && abs(ast.get.seed.y + 0.25f) < 1e-6f);
    assert(ast.get.taper == 0.6f && ast.get.taperPow == 2.0f);
    assert(abs(ast.get.heading - 0.3f) < 1e-6f);
    assert(abs(ast.get.motorPower - 77.0f) < 1e-6f);
    assert(abs(ast.get.lrGradient + 0.12f) < 1e-6f,
        "организменный LR-градиент читается в AST");

    assert(ast.get.segments.length == 1);
    const seg = ast.get.segments[0];
    assert(seg.fork);
    assert(seg.beams.length == 1);
    assert(seg.beams[0].start.kind == StartRefKind.base);
    assert(seg.beams[0].end.kind == EndRefKind.nearNode);
    assert(abs(seg.beams[0].end.delta.x - 1.0f) < 1e-6f);
    assert(abs(seg.beams[0].nodal - 0.05f) < 1e-6f);
    assert(abs(seg.beams[0].lefty - 0.6f) < 1e-6f);
    assert(abs(seg.beams[0].threshold - 1.5f) < 1e-6f,
        "порог билатеральности читается в AST");
    assert(abs(seg.beams[0].turn - 0.7f) < 1e-6f);

    assert(ast.get.anchors.length == 1);
    assert(ast.get.anchors[0].kind == AnchorKind.motorWheel);
    assert(ast.get.anchors[0].idx == 3);
    assert(abs(ast.get.anchors[0].radius - 0.28f) < 1e-6f,
        "радиус колеса читается в AST из токена wheelRadius");
}

unittest
{
    // Минимальный поток: необязательные токены опущены — организм без
    // полярности, порог билатеральности не блокирует асимметрию.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.heading, 0.0f);
    t ~= new Terminal!Tok(Tok.segStart);
    t ~= new Terminal!Tok(Tok.fork);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.nodal, 0.1f);
    t ~= new Terminal!Tok(Tok.lefty, 0.0f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.0f);

    auto ast = buildAst(t);
    assert(!ast.isNull);
    assert(ast.get.lrGradient == 0.0f, "без токена организм не имеет полярности");
    assert(ast.get.segments[0].beams[0].threshold == 0.0f,
        "без токена порог не блокирует асимметрию");
    assert(ast.get.motorPower == 0.0f, "без токена мотора нет");
    assert(beamAsymmetry(ast.get.lrGradient, ast.get.segments[0].beams[0]) > 0.0f);
}