module genetics.buggygrammar;

import std.math;
import std.random: Random, uniform;
import std.typecons: Nullable;
import dlib.math.vector;
import frame.frame;
import genetics.sge;

/**
 * Терминалы грамматики багги — «команды развития», которые `decode` выводит
 * из генома, а `frameFromTokens` выполняет в порядке следования, строя каркас
 * от первого узла. Часть токенов несёт значение (float/int в `payload`),
 * часть — пустые маркеры.
 */
enum Tok
{
    /// Начало балки: ссылка на последний созданный узел (маркер без значения).
    refLast,

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
}

private Terminal!Tok t(T)(Tok tok)
{
    return new Terminal!Tok(tok);
}

private Terminal!Tok t(T)(Tok tok, T val)
{
    return new Terminal!Tok(tok, val);
}

private NonTerminal nt(string name, Production[] productions)
{
    auto n = new NonTerminal(name);
    n.productions = productions;
    return n;
}

/// Маркерный токен без значения
private Terminal!Tok marker(Tok tok)
{
    assert(tok == Tok.refLast || tok == Tok.endNew || tok == Tok.endNear);

    return new Terminal!Tok(tok);
}

/**
 * Грамматика багги целиком.
 *
 * Первые три токена `coord` задают позицию первого узла. Затем идут балки,
 * затем — якоря (колёса). Узлы отдельно не генерируются: они появляются только
 * как концы балок. Каждая балка: старт — первый/последний созданный узел или
 * существующий по индексу, конец — вновь создаваемый (старт + смещение) или
 * существующий по индексу.
 *
 * Якоря генерируются после всех балок: это пара `anchorKind` + `refIdx`,
 * где `refIdx` ссылается на уже созданный узел.
 */
Grammar buggyGrammar()
{
    auto beamList_ = nt("beamList", null);

    auto idx = new Sampler!Tok("idx", Tok.refIdx, 64);
    auto destX = new Sampler!Tok("destX", Tok.coord, -1.5f, 1.5f);
    auto destY = new Sampler!Tok("destY", Tok.coord, -1.5f, 1.5f);
    auto destZ = new Sampler!Tok("destZ", Tok.coord, -1.5f, 1.5f);
    auto startX = new Sampler!Tok("startX", Tok.coord, -2.0f, 2.0f);
    auto startY = new Sampler!Tok("startY", Tok.coord, -2.0f, 2.0f);
    auto startZ = new Sampler!Tok("startZ", Tok.coord, -2.0f, 2.0f);
    auto radius = new Sampler!Tok("radius", Tok.radius, 0.02f, 0.06f);
    auto taper = new Sampler!Tok("taper", Tok.taper, 0.4f, 1.0f);
    auto taperPow = new Sampler!Tok("taperPow", Tok.taperPow, 0.5f, 4.0f);

    auto startRef = nt("startRef", [
        new Production([marker(Tok.refLast)]),
        new Production([idx]),
    ]);

    auto startPos = nt("startPos", [
        new Production([startX, startY, startZ, taper, taperPow]),
    ]);

    auto endRef = nt("endRef", [
        new Production([marker(Tok.endNew), destX, destY, destZ]),
        new Production([idx]),
        new Production([marker(Tok.endNear), destX, destY, destZ]),
    ]);

    auto beamKind = nt("beamKind", [
        new Production([t(Tok.beamKind, BeamKind.normal)]),
    ]);

    auto beam = nt("beam", [
        new Production([startRef, endRef, radius, beamKind]),
    ]);

    beamList_.productions = [
        new Production([beam, beamList_]),
        new Production([beam]),
    ];

    auto anchorList_ = nt("anchorList", null);

    auto anchorKind = nt("anchorKind", [
        new Production([t(Tok.anchorKind, AnchorKind.wheel)]),
        new Production([t(Tok.anchorKind, AnchorKind.motorWheel)]),
    ]);

    auto anchor = nt("anchor", [
        new Production([anchorKind, idx]),
    ]);

    anchorList_.productions = [
        new Production([anchor, anchorList_]),
        new Production([anchor]),
    ];

    // Маркер конца балок и начала якорей.
    auto anchorMarker = nt("anchorMarker", [
        new Production([new Terminal!Tok(Tok.anchors)]),
    ]);

    auto start = nt("frame", [
        new Production([startPos, beamList_, anchorMarker, anchorList_]),
    ]);

    auto symbols = [
        start, startPos, startX, startY, startZ, taper, taperPow, beamList_, beam, startRef,
        idx, endRef, destX, destY, destZ, radius, beamKind,
        anchorMarker, anchorList_, anchor, anchorKind,
    ];
    return new Grammar(start, symbols);
}

/**
 * Разобрать терминалы в каркас багги.
 *
 * Создаётся первый узел, затем балки по порядку до маркера `Tok.anchors`,
 * затем якоря (каждая пара `anchorKind` + `refIdx` прикрепляет колесо
 * к уже созданному узлу). Токены `refLast` ссылаются на последний созданный
 * узел, `refIdx` — на существующий по номеру, `endNew` (со смещениями)
 * создаёт новый узел на позиции старта + смещение, `endNear` — как `endNew`,
 * но растущий конец сливается с ближайшим существующим узлом в пределах
 * `mergeRadius` (так из правила роста сами возникают петли).
 *
 * Возврат — сам результат: `Nullable!Frame.isNull` означает ошибку разбора.
 */
Nullable!Frame frameFromTokens(const Terminal!Tok[] tokens)
{
    enum float mergeRadius = 0.15f;

    Frame result;

    // Первые три токена — абсолютная позиция первого узла.
    if (tokens.length < 3)
        return Nullable!Frame.init;
    if (tokens[0].tok != Tok.coord || tokens[1].tok != Tok.coord || tokens[2].tok != Tok.coord)
        return Nullable!Frame.init;
    auto seed = vec3(tokens[0].f, tokens[1].f, tokens[2].f);
    result.nodes ~= Node(seed);
    size_t last = 0;

    size_t i = 3;
    while (i < tokens.length && tokens[i].tok != Tok.anchors)
    {
        size_t start;
        switch (tokens[i].tok)
        {
            case Tok.refLast:
                start = last;
                ++i;
                break;
            case Tok.refIdx:
                start = tokens[i].i;
                if (start >= result.nodes.length)
                    return Nullable!Frame.init;
                ++i;
                break;
            default:
                return Nullable!Frame.init;
        }

        size_t end;
        switch (tokens[i].tok)
        {
            case Tok.endNew:
                if (i + 3 >= tokens.length)
                    return Nullable!Frame.init;
                if (tokens[i + 1].tok != Tok.coord
                    || tokens[i + 2].tok != Tok.coord
                    || tokens[i + 3].tok != Tok.coord)
                    return Nullable!Frame.init;
                auto dx = tokens[i + 1].f;
                auto dy = tokens[i + 2].f;
                auto dz = tokens[i + 3].f;
                result.nodes ~= Node(result.nodes[start].pos + vec3(dx, dy, dz));
                end = result.nodes.length - 1;
                last = end;
                i += 4;
                break;
            case Tok.endNear:
                if (i + 3 >= tokens.length)
                    return Nullable!Frame.init;
                if (tokens[i + 1].tok != Tok.coord
                    || tokens[i + 2].tok != Tok.coord
                    || tokens[i + 3].tok != Tok.coord)
                    return Nullable!Frame.init;
                auto tx = tokens[i + 1].f;
                auto ty = tokens[i + 2].f;
                auto tz = tokens[i + 3].f;
                auto target = result.nodes[start].pos + vec3(tx, ty, tz);
                // endNear — как endNew, но растущий конец сливается с ближайшим
                // существующим узлом (кроме старта) в пределах mergeRadius.
                // Из правила «расти до контакта» сами возникают петли и
                // самосборка каркаса.
                size_t best = size_t.max;
                auto bestD = mergeRadius;
                foreach (n; 0 .. result.nodes.length)
                {
                    if (n == start)
                        continue;
                    const d = distance(result.nodes[n].pos, target);
                    if (d < bestD)
                    {
                        bestD = d;
                        best = n;
                    }
                }
                if (best != size_t.max)
                    end = best;
                else
                {
                    result.nodes ~= Node(target);
                    end = result.nodes.length - 1;
                }
                last = end;
                i += 4;
                break;
            case Tok.refIdx:
                end = tokens[i].i;
                if (end >= result.nodes.length)
                    return Nullable!Frame.init;
                ++i;
                break;
            default:
                return Nullable!Frame.init;
        }

        if (tokens[i].tok != Tok.radius)
            return Nullable!Frame.init;
        auto radius = tokens[i].f;
        ++i;

        if (tokens[i].tok != Tok.beamKind)
            return Nullable!Frame.init;
        auto beamKind = cast(BeamKind) tokens[i].i;
        ++i;

        result.beams ~= Beam(start, end, radius, beamKind);
    }

    if (i >= tokens.length)
        return Nullable!Frame(result);

    // Маркер `Tok.anchors` — переход к якорям.
    ++i;
    while (i < tokens.length)
    {
        if (tokens[i].tok != Tok.anchorKind)
            return Nullable!Frame.init;
        auto kind = cast(AnchorKind) tokens[i].i;
        ++i;

        if (i >= tokens.length || tokens[i].tok != Tok.refIdx)
            return Nullable!Frame.init;
        // Индекс узла, как и кодоны SGE, заворачивается по числу узлов:
        // случайный индекс за пределами каркаса прижимается к существующему узлу,
        // а не роняет весь кадр.
        auto n = cast(size_t) tokens[i].i % result.nodes.length;
        ++i;

        result.anchors ~= Anchor(n, kind);
    }
    return Nullable!Frame(result);
}

/**
 * Структурная валидность каркаса: узлы в границах, без вырожденных балок,
 * якоря — на валидных узлах, каркас связен.
 */
Nullable!Frame isValidFrame(Frame f)
{
    if (f.nodes.length == 0 || f.beams.length == 0)
        return Nullable!Frame.init;
    foreach (b; f.beams)
    {
        if (b.a >= f.nodes.length || b.b >= f.nodes.length)
            return Nullable!Frame.init;
        const pa = f.nodes[b.a].pos;
        const pb = f.nodes[b.b].pos;
        if (b.a == b.b || distance(pa, pb) < 1e-4f)
            return Nullable!Frame.init;
    }

    // Дубли якорей на одном узле допускаются — это вырожденный случай, который
    // отсеется на этапе физики/фитнеса, а не на этапе синтаксиса.
    foreach (a; f.anchors)
        if (a.node >= f.nodes.length)
            return Nullable!Frame.init;

    if (!isConnected(f))
        return Nullable!Frame.init;
    return Nullable!Frame(f);
}

/// Расшифровать геном из грамматики багги в готовый каркас багги.
/// Значение-результат сам говорит об успехе: `Nullable!Frame.isNull`
/// означает, что декодирование, разбор или валидация не прошли.
Nullable!Frame develop(const Grammar gr, const Genotype g)
{
    auto tokens = decode!Tok(gr, g);
    if (tokens is null)
        return Nullable!Frame.init;
    auto frame = frameFromTokens(applyMorph(tokens));
    if (frame.isNull)
        return Nullable!Frame.init;
    return isValidFrame(frame.get);
}

/**
 * Морфоген-градиент толщины: радиус каждой балки масштабируется вдоль
 * порядка построения. Первая балка не меняется, последняя — в `taper` раз,
 * промежуточные — по степенному закону `1 + (taper - 1) * frac^taperPow`,
 * где `frac = j / (n - 1)`. Параметры градиента читаются из токенов
 * `Tok.taper` / `Tok.taperPow` (по одному на каркас, задаются в `startPos`)
 * и из потока убираются, чтобы интерпретатор их не видел. Нейтральный
 * вариант (taper == 1.0) не меняет геометрию.
 */
private Terminal!Tok[] applyMorph(Terminal!Tok[] tokens)
{
    float taper = 1.0f;
    float taperPow = 1.0f;

    Terminal!Tok[] rest;
    foreach (tok; tokens)
    {
        switch (tok.tok)
        {
            case Tok.taper:
                taper = tok.f;
                continue;
            case Tok.taperPow:
                taperPow = tok.f;
                continue;
            default:
                rest ~= tok;
        }
    }
    if (rest.length == tokens.length)
        return rest;

    size_t n;
    foreach (tok; rest)
        if (tok.tok == Tok.radius)
            ++n;
    if (n < 2)
        return rest;

    size_t j;
    foreach (i, ref tok; rest)
    {
        if (tok.tok != Tok.radius)
            continue;
        const frac = cast(float) j / (n - 1);
        const factor = 1.0f + (taper - 1.0f) * pow(frac, taperPow);
        rest[i] = new Terminal!Tok(Tok.radius, tok.f * factor);
        ++j;
    }
    return rest;
}

/**
 * Один шаг мутации генома с проверкой развития.
 *
 * Обычно — точечные правки (`mutate`). С вероятностью 25% — индельная
 * мутация (`mutateIndel`), единственная, что меняет длины генов и, значит,
 * число балок и колёс. Индел принимается, только если он действительно
 * изменил структуру: иначе редкая структурная правка тонет среди
 * геометрических, и каркас никогда не растёт.
 *
 * Возврат — сам результат: `Nullable!Genotype.isNull` означает, что подходящий
 * мутант не нашёлся.
 */
Nullable!Genotype mutateStep(const Grammar gr, const Genotype genome, ref Random rnd)
{
    const current = develop(gr, genome);
    if (current.isNull)
        return Nullable!Genotype.init;

    const structural = uniform(0.0f, 1.0f, rnd) < 0.25f;
    foreach (_; 0 .. 100)
    {
        auto candidate = genome.dup;
        if (structural)
            mutateIndel(candidate, 1, rnd);
        else
            mutate(candidate, 1 + uniform(0u, 3u, rnd), rnd);

        auto may = develop(gr, candidate);
        if (may.isNull)
            continue;
        if (structural
            && may.get.beams.length == current.get.beams.length
            && may.get.anchors.length == current.get.anchors.length)
            continue;

        return Nullable!Genotype(candidate);
    }
    return Nullable!Genotype.init;
}

unittest
{
    import std.random : Random;

    auto gr = buggyGrammar();
    auto rnd = Random(42);

    size_t valid, decodeOk;
    foreach (_; 0 .. 5000)
    {
        auto g = randomGenotype(gr, 8, rnd);
        assert(g.genes.length == gr.symbols.length);

        auto tokens = decode!Tok(gr, g);
        if (tokens is null)
            continue;
        ++decodeOk;
        assert(tokens.length > 0);

        auto f = frameFromTokens(applyMorph(tokens));
        if (f.isNull)
            continue;

        if (!isValidFrame(f.get).isNull)
            ++valid;
    }

    // Декодирование сходится почти всегда; значительная доля геномов даёт
    // валидный каркас (остальные отбрасываются самокоррекцией — ссылки на
    // узлы, которых ещё нет, вырожденные балки).
    assert(decodeOk > 3000);
    assert(valid > 20);

    // Кроссинговер сохраняет число генов (по одному на нетерминал).
    auto g1 = randomGenotype(gr, 8, rnd);
    auto g2 = randomGenotype(gr, 8, rnd);
    auto child = crossover(g1, g2, rnd);
    assert(child.genes.length == gr.symbols.length);
}

unittest
{
    // endNear: растущий конец сливается с ближайшим существующим узлом
    // в пределах допуска — возникает замкнутая петля без нового узла.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNew);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    t ~= new Terminal!Tok(Tok.coord, -0.95f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = frameFromTokens(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 2,
        "endNear должен слиться с узлом 0, а не создавать новый");
    assert(f.get.beams.length == 2);
    assert(f.get.beams[1].a == 1 && f.get.beams[1].b == 0,
        "вторая балка замыкает петлю на существующий узел");
    assert(!isValidFrame(f.get).isNull, "замкнутая петля остаётся связной");
}

unittest
{
    // endNear далеко от структуры ведёт себя как endNew — новый узел.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.refLast);
    t ~= new Terminal!Tok(Tok.endNear);
    t ~= new Terminal!Tok(Tok.coord, 1.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.radius, 0.04f);
    t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    t ~= new Terminal!Tok(Tok.anchors);

    auto f = frameFromTokens(t);
    assert(!f.isNull);
    assert(f.get.nodes.length == 2);
    assert(f.get.beams.length == 1);
}

unittest
{
    // Морфоген-градиент: радиус балок масштабируется вдоль порядка
    // построения (0.5 в конце), морфоген-токены убираются из потока.
    Terminal!Tok[] t;
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.coord, 0.0f);
    t ~= new Terminal!Tok(Tok.taper, 0.5f);
    t ~= new Terminal!Tok(Tok.taperPow, 1.0f);
    foreach (_; 0 .. 2)
    {
        t ~= new Terminal!Tok(Tok.refLast);
        t ~= new Terminal!Tok(Tok.endNew);
        t ~= new Terminal!Tok(Tok.coord, 1.0f);
        t ~= new Terminal!Tok(Tok.coord, 0.0f);
        t ~= new Terminal!Tok(Tok.coord, 0.0f);
        t ~= new Terminal!Tok(Tok.radius, 0.06f);
        t ~= new Terminal!Tok(Tok.beamKind, cast(int) BeamKind.normal);
    }
    t ~= new Terminal!Tok(Tok.anchors);

    auto m = applyMorph(t);
    size_t r;
    foreach (tok; m)
        if (tok.tok == Tok.radius)
        {
            if (r == 0)
                assert(tok.f == 0.06f, "первая балка не меняется");
            else
                assert(tok.f == 0.03f, "последняя балка сужается в taper раз");
            ++r;
        }
    assert(r == 2);

    size_t morph;
    foreach (tok; m)
        if (tok.tok == Tok.taper || tok.tok == Tok.taperPow)
            ++morph;
    assert(morph == 0, "морфоген-токены убираются из потока");
}

unittest
{
    // Вырожденная балка и разорванный каркас отбрасываются.
    Frame f;
    f.nodes = [
        Node(vec3(0.0f, 0.0f, 0.0f)),
        Node(vec3(0.0f, 1.0f, 0.0f)),
    ];
    f.beams = [Beam(0, 0, 0.04f)];
    f.anchors = [Anchor(0, AnchorKind.wheel)];
    assert(isValidFrame(f).isNull);

    // Простейший валидный каркас: пара узлов с балкой и колёсами.
    Frame g;
    g.nodes = [
        Node(vec3(0.6f, 0.7f, 0.3f)),
        Node(vec3(0.6f, -0.7f, 0.3f)),
    ];
    g.beams = [Beam(0, 1, 0.05f)];
    g.anchors = [
        Anchor(0, AnchorKind.wheel),
        Anchor(1, AnchorKind.motorWheel),
    ];
    assert(!isValidFrame(g).isNull);
}