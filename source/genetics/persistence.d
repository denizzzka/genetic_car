module genetics.persistence;

import std.exception : enforce;
import std.file : read, write;
import std.format : format;
import std.math : round;

import dlib.math.vector : vec3;

import frame.frame : Anchor, AnchorKind, Beam, BeamKind, EphemeralBeam,
    Frame, Node, frameError, frameVersion;
import genetics.chromosome : Chromosome, SegmentGenes, alleles,
    chromosomeError, chromosomeVersion, segmentGeneCount, speciesGeneCount;
import genetics.growth : develop, growthGrammarVersion;

/// Версия файла багги: старая версия не читается молча.
enum uint buggyFileVersion = 1;

/// "GCAR" в little-endian.
private enum uint buggyFileMagic = 0x52414347;

/// Загруженный багги: каркас используется как есть, геном - для разведения.
struct SavedBuggy
{
    Chromosome chromosome;
    Frame frame;
    uint grammarVersion;
    ulong frameHash;
}

/// Сохранить геном и каркас в файл. Каркас обязателен: при загрузке он
/// используется напрямую, без `develop`.
void saveBuggy(string path, const Chromosome chromosome, const Frame frame)
{
    const genomeWhy = chromosomeError(chromosome);
    enforce(genomeWhy is null, "геном: " ~ genomeWhy);
    const frameWhy = frameError(frame);
    enforce(frameWhy is null, "каркас: " ~ frameWhy);

    const ulong hash = frameFingerprint(frame);
    ubyte[] bytes;
    putMagic(bytes);
    putU32(bytes, buggyFileVersion);
    putU32(bytes, chromosomeVersion);
    putU32(bytes, growthGrammarVersion);
    putU32(bytes, frameVersion);
    putU32(bytes, cast(uint) speciesGeneCount);
    putU32(bytes, cast(uint) segmentGeneCount);
    putU32(bytes, cast(uint) chromosome.segments.length);
    putU64(bytes, hash);
    static foreach (i; 0 .. speciesGeneCount)
        putF32(bytes, chromosome.species.tupleof[i]);
    foreach (seg; chromosome.segments)
        static foreach (i; 0 .. segmentGeneCount)
            putF32(bytes, seg.tupleof[i]);
    bytes ~= serializeFrame(frame);
    write(path, bytes);
}

/// Прочитать багги. Проверяет версии, гены, каркас и хеш; при несовпадении
/// бросает исключение, а не отдаёт другой каркас.
SavedBuggy loadBuggy(string path)
{
    auto raw = cast(ubyte[]) read(path);
    Reader r = Reader(raw);
    enforce(r.u32() == buggyFileMagic, "это не файл багги");
    const uint fileVersion = r.u32();
    enforce(fileVersion == buggyFileVersion,
        format("версия файла багги %s не поддерживается", fileVersion));
    const uint chrVersion = r.u32();
    enforce(chrVersion == chromosomeVersion,
        format("версия генома %s не поддерживается", chrVersion));
    const uint grammar = r.u32();
    enforce(grammar == growthGrammarVersion,
        format("версия грамматики %s не поддерживается", grammar));
    const uint frameVer = r.u32();
    enforce(frameVer == frameVersion,
        format("версия каркаса %s не поддерживается", frameVer));
    const uint speciesCount = r.u32();
    const uint segmentCount = r.u32();
    enforce(speciesCount == speciesGeneCount && segmentCount == segmentGeneCount,
        "раскладка генов не совпадает с текущей");
    const uint phases = r.u32();
    const ulong hash = r.u64();

    Chromosome chromosome;
    static foreach (i; 0 .. speciesGeneCount)
        chromosome.species.tupleof[i] = r.f32();
    chromosome.segments = new SegmentGenes[phases];
    foreach (ref seg; chromosome.segments)
        static foreach (i; 0 .. segmentGeneCount)
            seg.tupleof[i] = r.f32();
    const genomeWhy = chromosomeError(chromosome);
    enforce(genomeWhy is null, "геном: " ~ genomeWhy);

    Frame frame;
    const uint nodeCount = r.u32();
    frame.nodes = new Node[nodeCount];
    foreach (ref n; frame.nodes)
        n.pos = vec3(r.f32(), r.f32(), r.f32());
    const uint beamCount = r.u32();
    frame.beams = new EphemeralBeam[beamCount];
    foreach (i; 0 .. beamCount)
    {
        const uint a = r.u32();
        const uint b = r.u32();
        const ubyte isBeam = r.u8();
        const float radius = r.f32();
        const ubyte kind = r.u8();
        enforce(isBeam <= 1, "флаг балки негоден");
        frame.beams[i] = isBeam
            ? new Beam(a, b, radius, cast(BeamKind) kind)
            : new EphemeralBeam(a, b);
    }
    const uint anchorCount = r.u32();
    frame.anchors = new Anchor[anchorCount];
    foreach (ref a; frame.anchors)
    {
        a.node = r.u32();
        a.kind = cast(AnchorKind) r.u8();
        a.radius = r.f32();
    }
    frame.motorPower = r.f32();
    r.expectEnd();

    const frameWhy = frameError(frame);
    enforce(frameWhy is null, "каркас: " ~ frameWhy);
    enforce(frameFingerprint(frame) == hash, "хеш каркаса не совпадает");
    return SavedBuggy(chromosome, frame, grammar, hash);
}

/// Проверить сохранённый каркас повторным выращиванием из генома.
bool verifyByRegrow(const SavedBuggy saved, out string why)
{
    const grown = develop(saved.chromosome);
    if (grown.isNull)
    {
        why = "геном не вырастает в каркас";
        return false;
    }
    if (frameFingerprint(grown.get) != saved.frameHash)
    {
        why = "каркас разошёлся с сохранённым";
        return false;
    }
    why = null;
    return true;
}

/// Отпечаток каркаса по квантованным значениям: сравнивает сохранённый и
/// свежевыращенный каркас, не ловя шум libm.
private ulong frameFingerprint(const Frame frame)
{
    const bytes = serializeFrameQuantized(frame);
    ulong h = 1469598103934665603UL;
    foreach (c; bytes)
    {
        h ^= c;
        h *= 1099511628211UL;
    }
    return h;
}

private ubyte[] serializeFrame(const Frame frame)
{
    ubyte[] bytes;
    putU32(bytes, cast(uint) frame.nodes.length);
    foreach (n; frame.nodes)
    {
        putF32(bytes, n.pos.x);
        putF32(bytes, n.pos.y);
        putF32(bytes, n.pos.z);
    }
    putU32(bytes, cast(uint) frame.beams.length);
    foreach (b; frame.beams)
    {
        const beam = cast(Beam) b;
        putU32(bytes, cast(uint) b.a);
        putU32(bytes, cast(uint) b.b);
        bytes ~= cast(ubyte) (beam !is null);
        putF32(bytes, beam is null ? 0.0f : beam.radius);
        bytes ~= cast(ubyte) (beam is null ? 0 : beam.kind);
    }
    putU32(bytes, cast(uint) frame.anchors.length);
    foreach (a; frame.anchors)
    {
        putU32(bytes, cast(uint) a.node);
        bytes ~= cast(ubyte) a.kind;
        putF32(bytes, a.radius);
    }
    putF32(bytes, frame.motorPower);
    return bytes;
}

private ubyte[] serializeFrameQuantized(const Frame frame)
{
    ubyte[] bytes;
    putU32(bytes, cast(uint) frame.nodes.length);
    foreach (n; frame.nodes)
    {
        putQuantized(bytes, n.pos.x);
        putQuantized(bytes, n.pos.y);
        putQuantized(bytes, n.pos.z);
    }
    putU32(bytes, cast(uint) frame.beams.length);
    foreach (b; frame.beams)
    {
        const beam = cast(Beam) b;
        putU32(bytes, cast(uint) b.a);
        putU32(bytes, cast(uint) b.b);
        bytes ~= cast(ubyte) (beam !is null);
        putQuantized(bytes, beam is null ? 0.0f : beam.radius);
        bytes ~= cast(ubyte) (beam is null ? 0 : beam.kind);
    }
    putU32(bytes, cast(uint) frame.anchors.length);
    foreach (a; frame.anchors)
    {
        putU32(bytes, cast(uint) a.node);
        bytes ~= cast(ubyte) a.kind;
        putQuantized(bytes, a.radius);
    }
    putQuantized(bytes, frame.motorPower);
    return bytes;
}

private void putQuantized(ref ubyte[] bytes, float value)
{
    putU32(bytes, cast(uint) cast(int) round(value * 100000.0f));
}

private void putMagic(ref ubyte[] bytes)
{
    bytes ~= cast(ubyte) 'G';
    bytes ~= cast(ubyte) 'C';
    bytes ~= cast(ubyte) 'A';
    bytes ~= cast(ubyte) 'R';
}

private void putU32(ref ubyte[] bytes, uint v)
{
    bytes ~= cast(ubyte) (v & 0xFF);
    bytes ~= cast(ubyte) ((v >> 8) & 0xFF);
    bytes ~= cast(ubyte) ((v >> 16) & 0xFF);
    bytes ~= cast(ubyte) ((v >> 24) & 0xFF);
}

private void putU64(ref ubyte[] bytes, ulong v)
{
    putU32(bytes, cast(uint) (v & 0xFFFFFFFF));
    putU32(bytes, cast(uint) (v >> 32));
}

private void putF32(ref ubyte[] bytes, float v)
{
    union Bits
    {
        float f;
        uint u;
    }
    Bits bits;
    bits.f = v;
    putU32(bytes, bits.u);
}

private struct Reader
{
    const(ubyte)[] data;
    size_t pos;

    ubyte u8()
    {
        enforce(pos < data.length, "файл обрезан");
        return cast(ubyte) data[pos++];
    }

    uint u32()
    {
        enforce(pos + 4 <= data.length, "файл обрезан");
        const uint v = cast(uint) data[pos]
            | (cast(uint) data[pos + 1] << 8)
            | (cast(uint) data[pos + 2] << 16)
            | (cast(uint) data[pos + 3] << 24);
        pos += 4;
        return v;
    }

    ulong u64()
    {
        const ulong lo = u32();
        const ulong hi = u32();
        return lo | (hi << 32);
    }

    float f32()
    {
        union Bits
        {
            float f;
            uint u;
        }
        Bits bits;
        bits.u = u32();
        return bits.f;
    }

    void expectEnd()
    {
        enforce(pos == data.length, "лишние байты в файле");
    }
}

unittest
{
    import std.exception : assertThrown;
    import std.file : exists, read, remove, write;
    import genetics.growth : develop;
    import genetics.initial_data : startChromosome;

    const chromosome = startChromosome();
    const grown = develop(chromosome);
    assert(!grown.isNull, "основатель должен вырасти");
    const Frame frame = grown.get;

    enum string path = "/tmp/opencode/buggy-persistence.gcar";
    scope (exit)
        if (exists(path))
            remove(path);

    saveBuggy(path, chromosome, frame);
    const loaded = loadBuggy(path);
    assert(alleles(loaded.chromosome) == alleles(chromosome),
        "геном не пережил файл");
    assert(loaded.frame.nodes.length == frame.nodes.length,
        "каркас не пережил файл");
    assert(frameFingerprint(loaded.frame) == frameFingerprint(frame),
        "отпечаток каркаса изменился");

    string why;
    assert(verifyByRegrow(loaded, why), why);

    // Порча файла ловится, а не отдаёт половину каркаса.
    auto raw = cast(ubyte[]) read(path);
    raw[$ / 2] ^= 0xFF;
    write(path, raw);
    assertThrown!Exception(loadBuggy(path));

    // Геном вне границ не сохраняется.
    Chromosome bad = startChromosome();
    bad.species.beamRadius = 1.0f;
    assertThrown!Exception(saveBuggy(path, bad, frame));
}
