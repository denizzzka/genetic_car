module genetics.buggyast;

import std.math;
import std.typecons: Nullable;
import dlib.math.vector;
import frame.frame;
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

    /// Nodal — активатор асимметрии twin-пары (float). Знак задаёт, в какую
    /// сторону уводятся twin-балки, величина — силу сдвига. Маленькое |nodal|
    /// почти не ломает симметрию; сильный активатор усиливается, но гасится
    /// ингибитором `lefty` — как в LR-генезе позвоночных. Одиночные
    /// медианные балки (без twin) активатор не затрагивает совсем.
    nodal,

    /// Lefty — ингибитор асимметрии (float, ≥ 0). Нелинейно гасит активатор:
    /// чем сильнее `nodal`, тем больше подавление (`lefty·nodal²`) — так
    /// асимметрия пары остаётся малой и самоограниченной, а не «разносит»
    /// структуру. Ноль — активатор действует в полную силу.
    lefty,

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
    /// или дельта `destX/destY/destZ` относительно старта балки.
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

    /// Морфоген: во сколько раз сужается толщина к последней балке
    /// (float, 0.4..1.0). Токен изымается из потока до интерпретации.
    taper,

    /// Морфоген: показатель степени кривой градиента толщины
    /// (float, 0.5..4.0). Токен изымается из потока до интерпретации.
    taperPow,

    /// Turtle: начальный заголовок построения (float, радианы). Дельты
    /// `endNew`/`endNear` интерпретируются в системе заголовка: `coord` X —
    /// вперёд по заголовку, `coord` Y — вправо от него, Z — вверх.
    heading,

    /// Turtle: приращение заголовка после балки (float, радианы).
    /// Накапливается в общее направление построения.
    turn,
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
    BeamKind kind;
    float turn;
}

struct SegmentAst
{
    bool fork;
    float axis;
    BeamAst[] beams;
}

struct AnchorAst
{
    AnchorKind kind;
    size_t idx;
}

struct Ast
{
    vec3 seed;
    float heading;
    float taper = 1.0f;
    float taperPow = 1.0f;
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

        ast.anchors ~= a;
    }
    return Nullable!Ast(ast);
}

/**
 * Эффективная асимметрия twin-пары от активатора Nodal и ингибитора Lefty.
 *
 * Ответная кривая активатор-ингибитор: `nodal` усиливает сдвиг, а `lefty`
 * гасит его квадратично (`lefty·nodal²`), поэтому при сильном активаторе
 * подавление растёт — асимметрия мала и самограничена, как в LR-генезе
 * позвоночных (билатеральность сохраняется, но допускает эволюцию).
 * Для медианных одиночных балок (без twin) активатор не применяется вовсе.
 */
float forkAsymmetry(float nodal, float lefty)
{
    return nodal / (1.0f + lefty * nodal * nodal);
}

unittest
{
    // Nodal/Lefty: слабый активатор почти симметричен, сильный гасится.
    assert(abs(forkAsymmetry(0.0f, 0.0f)) < 1e-7f, "нулевой nodal — точная симметрия");
    const strong = forkAsymmetry(0.3f, 0.0f);
    assert(strong > 0.2f, "без ингибитора активатор действует в полную силу");
    const damped = forkAsymmetry(0.3f, 1.0f);
    assert(damped > 0.0f && damped < strong,
        "ингибитор гасит активатор нелинейно");
    assert(forkAsymmetry(-0.1f, 0.0f) < 0.0f, "знак активатора переворачивает сдвиг");
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
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.turn, 0.7f);
    t ~= new Terminal!Tok(Tok.anchors);
    t ~= new Terminal!Tok(Tok.anchorKind, cast(int) AnchorKind.motorWheel);
    t ~= new Terminal!Tok(Tok.refIdx, cast(int) 3);

    auto ast = buildAst(t);
    assert(!ast.isNull);
    assert(abs(ast.get.seed.x - 0.5f) < 1e-6f && abs(ast.get.seed.y + 0.25f) < 1e-6f);
    assert(ast.get.taper == 0.6f && ast.get.taperPow == 2.0f);
    assert(abs(ast.get.heading - 0.3f) < 1e-6f);

    assert(ast.get.segments.length == 1);
    const seg = ast.get.segments[0];
    assert(seg.fork);
    assert(seg.beams.length == 1);
    assert(seg.beams[0].start.kind == StartRefKind.base);
    assert(seg.beams[0].end.kind == EndRefKind.nearNode);
    assert(abs(seg.beams[0].end.delta.x - 1.0f) < 1e-6f);
    assert(abs(seg.beams[0].nodal - 0.05f) < 1e-6f);
    assert(abs(seg.beams[0].lefty - 0.6f) < 1e-6f);
    assert(abs(seg.beams[0].turn - 0.7f) < 1e-6f);

    assert(ast.get.anchors.length == 1);
    assert(ast.get.anchors[0].kind == AnchorKind.motorWheel);
    assert(ast.get.anchors[0].idx == 3);
}