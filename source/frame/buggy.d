module frame.buggy;

import std.algorithm : filter;
import std.range : walkLength;
import dlib.math.vector;
import frame.frame;

Frame buggyFrame()
{
    Frame f;

    size_t node(vec3 pos, AnchorKind kind = AnchorKind.none)
    {
        f.nodes ~= Node(pos, kind);
        return f.nodes.length - 1;
    }

    void beam(size_t a, size_t b, float radius = 0.04f, BeamKind kind = BeamKind.normal)
    {
        f.beams ~= Beam(a, b, radius, kind);
    }

    const railBack = node(vec3(0.50f, -1.10f, 0.18f));
    const railMidBack = node(vec3(0.50f, -0.72f, 0.18f));
    const railMid = node(vec3(0.50f, 0.05f, 0.18f));
    const railMidFront = node(vec3(0.50f, 0.72f, 0.18f));
    const railFront = node(vec3(0.50f, 1.05f, 0.18f));

    const hoopTop = node(vec3(0.44f, -0.55f, 1.05f));
    const roofBack = node(vec3(0.42f, -0.15f, 1.05f));
    const roofFront = node(vec3(0.40f, 0.45f, 0.95f));
    const dashTop = node(vec3(0.44f, 0.72f, 0.35f));
    const noseTop = node(vec3(0.46f, 0.95f, 0.30f));

    const wheelRear = node(vec3(0.72f, -0.95f, 0.10f), AnchorKind.wheel);
    const wheelFront = node(vec3(0.72f, 0.75f, 0.10f), AnchorKind.wheel);
    const shockFront = node(vec3(0.55f, 0.80f, 0.60f), AnchorKind.shock);
    const shockRear = node(vec3(0.55f, -0.80f, 0.70f), AnchorKind.shock);
    const springFront = node(vec3(0.60f, 0.82f, 0.12f), AnchorKind.spring);
    const springRear = node(vec3(0.60f, -0.82f, 0.12f), AnchorKind.spring);
    const motor = node(vec3(0.00f, -0.05f, 0.50f), AnchorKind.motor);
    const axle = node(vec3(0.00f, -0.70f, 0.15f), AnchorKind.axle);

    const floorMid = node(vec3(0.00f, -0.35f, 0.18f));
    const floorFront = node(vec3(0.00f, 0.55f, 0.18f));
    const roofCenter = node(vec3(0.00f, -0.10f, 1.02f));

    const railRadius = 0.05f;
    const cageRadius = 0.045f;
    const noseRadius = 0.035f;
    const susRadius = 0.03f;
    const crossRadius = 0.04f;

    beam(railBack, railMidBack, railRadius);
    beam(railMidBack, railMid, railRadius);
    beam(railMid, railMidFront, railRadius);
    beam(railMidFront, railFront, railRadius);

    beam(railMidBack, hoopTop, cageRadius);
    beam(hoopTop, roofBack, cageRadius);
    beam(roofBack, roofFront, cageRadius);
    beam(roofFront, dashTop, cageRadius);
    beam(dashTop, railMidFront, cageRadius);
    beam(hoopTop, dashTop, cageRadius);

    beam(railMidFront, noseTop, noseRadius);
    beam(noseTop, railFront, noseRadius);
    beam(railFront, railMidFront, noseRadius);

    beam(shockRear, roofBack, susRadius);
    beam(shockRear, railMidBack, susRadius);
    beam(shockFront, roofFront, susRadius);
    beam(shockFront, dashTop, susRadius);

    beam(wheelFront, springFront, susRadius);
    beam(springFront, railMidFront, susRadius);
    beam(wheelFront, railMidFront, susRadius);
    beam(wheelRear, springRear, susRadius);
    beam(springRear, railMidBack, susRadius);
    beam(wheelRear, railMidBack, susRadius);

    beam(railMid, floorMid, crossRadius);
    beam(railMidFront, floorFront, crossRadius);
    beam(railMidFront, floorMid, crossRadius);
    beam(railBack, axle, crossRadius);
    beam(axle, railMidBack, crossRadius);
    beam(railFront, floorFront, crossRadius);

    beam(roofBack, roofCenter, crossRadius);
    beam(roofCenter, roofFront, crossRadius);
    beam(floorMid, floorFront, crossRadius);

    beam(motor, railMid, susRadius);
    beam(motor, hoopTop, susRadius);
    beam(motor, railMidBack, susRadius);

    beam(railFront, roofBack, crossRadius, BeamKind.cross);
    beam(railBack, dashTop, crossRadius, BeamKind.cross);

    return f;
}

size_t anchorCount(const Frame f)
{
    return f.nodes.filter!(n => n.kind != AnchorKind.none).walkLength();
}

size_t planeNodeCount(const Frame f)
{
    return f.nodes.filter!(n => isOnPlane(n.pos)).walkLength();
}

unittest
{
    const f = buggyFrame();

    assert(f.nodes.length > 10);
    assert(f.beams.length > 10);
    assert(anchorCount(f) >= 6);

    const full = mirrorClosure(f);
    assert(isSymmetric(full));
    assert(full.nodes.length == 2 * f.nodes.length - planeNodeCount(f));
    assert(full.totalBeamLength > 0.0f);
}