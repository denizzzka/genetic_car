module genetics.buggyast;

import std.math;
import dlib.math.vector;
import frame.frame;

enum StartRefKind { last, idx }
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
    BeamKind kind;
    float turn;
}

struct SegmentAst
{
    bool fork;
    float forkDelta;
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