/**
 * Wheel of the frame: the axle always lies across the course — the disc plane
 * faces forward, so the wheel rolls straight from the first step instead of
 * scrubbing sideways. A revolute joint welds the wheel to the master frame.
 *
 * The module is intentionally leaf (does not import car.d): the joint works
 * through the plain `NewtonRigidBody`, and the axle direction derives from the
 * frame basis alone, so wheel physics does not depend on how the run is built.
 */
module physics_world.wheel;

import std.math;

import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;

import dagon.ext.newton;

/// Default wheel radius, m. Value the `Anchor` gets when the `wheelRadius`
/// gene is absent (hand-built frames without it). Each wheel's genetic radius
/// scales relative to this base: tire inner radius and width grow/shrink
/// proportionally.
enum float defaultWheelRadius = 0.3f;

/// Wheel axle that keeps the wheel facing forward: across the course
/// (`cross(up, forward)` — the right ray of the frame basis), so the disc plane
/// lines up with forward and the wheel rolls straight no matter how the node
/// beam is directed. Mirrored by the wheel's side (`nodePos.x`), so the two
/// track rows roll forward together. `forward`/`up` form the orthonormal frame
/// basis; `nodePos` is the anchor node in car coordinates.
vec3 wheelAxle(const vec3 forward, const vec3 up, const vec3 nodePos)
{
    const vec3 across = cross(up, forward);
    return nodePos.x < 0.0f ? -across : across;
}

/// Own hub: the wheel and the beam share the anchor node — the axle legitimately
/// passes through the wheel, so contact with it is not a failure.
bool isOwnWheel(size_t wheelNode, size_t beamA, size_t beamB)
{
    return wheelNode == beamA || wheelNode == beamB;
}

/**
 * One-ended wheel axle: a revolute joint on Newton's custom joint. Unlike
 * BallConstraint (a ball joint — pivot fixed, but all three rotations free,
 * wheel wobbles), here only the spin around the wheel axle is free and the two
 * cross-axis rotations are locked — the wheel rolls on a fixed axle.
 *
 * The hinge axis is the wheel's local Y (cylinder axis), wherever it points:
 * the direction is set by the wheel's build-time rotation (`wheelAxle`), the
 * joint never sees it.
 *
 * The build mirrors the reference Newton hinge (`dCustomHinge::SubmitConstraints`):
 * three linear rows hold the pivot, two angular rows around the PARENT's cross
 * axes kill the wheel-axle tilt. The angle comes from `calculateAngle`
 * (wheel axis projected onto a plane, atan2), as in the engine — the relative
 * quaternion cannot be used (sign/frame drift).
 */
final class WheelAxleJoint : NewtonUserConstraint
{
    private NewtonRigidBody wheel_;
    private NewtonRigidBody master_;
    /// Pivot of the joint in the master frame's local coordinates (Newton world).
    private vec3 pivotMasterLocal_;
    /// Joint axle and its two cross directions in the master's local coords.
    private vec3 masterPinLocal_;
    private vec3 masterUpLocal_;
    private vec3 masterRightLocal_;

    this(NewtonRigidBody wheel, NewtonRigidBody master, vec3 pivotMasterLocal)
    {
        super(master.world, wheel, master, 6);
        wheel_ = wheel;
        master_ = master;
        pivotMasterLocal_ = pivotMasterLocal;

        // Master joint frame: axle = the wheel's live world axle expressed in
        // master local coords. The master is unrotated at build time, but we
        // compute it honestly — in case the frame was placed already.
        Matrix4x4f wm, mm;
        NewtonBodyGetMatrix(wheel.newtonBody, wm.arrayof.ptr);
        NewtonBodyGetMatrix(master.newtonBody, mm.arrayof.ptr);
        const vec3 axleWorld = wm.rotate(Vector3f(0.0f, 1.0f, 0.0f));
        Quaternionf masterTrue = Quaternionf.fromMatrix(mm).conj;
        masterPinLocal_ = masterTrue.conj.rotate(axleWorld);

        const vec3 tmp = abs(masterPinLocal_.x) < 0.9f
            ? Vector3f(1.0f, 0.0f, 0.0f) : Vector3f(0.0f, 1.0f, 0.0f);
        masterUpLocal_ = unit(cross(masterPinLocal_, tmp));
        masterRightLocal_ = unit(cross(masterPinLocal_, masterUpLocal_));
    }

    /// Unit vector or zero — the axle is always defined, but guard the /0.
    private static vec3 unit(vec3 v)
    {
        const float l = v.length;
        return l < 1e-5f ? Vector3f(0.0f, 0.0f, 0.0f) : v / l;
    }

    /// Signed angle between `dir` and `cosDir` in the plane with normal `sinDir`
    /// (port of `dCustomJoint::CalculateAngle`).
    private static float calculateAngle(vec3 dir, vec3 cosDir, vec3 sinDir)
    {
        const vec3 projectDir = dir - sinDir * dot(dir, sinDir);
        const float cosAngle = dot(projectDir, cosDir);
        const float sinAngle = dot(sinDir, cross(projectDir, cosDir));
        return atan2(sinAngle, cosAngle);
    }

    override void submit(float timestep, int threadIndex)
    {
        // Live body matrices at solve time (not the wrapper cache — it lags).
        Matrix4x4f m0, m1;
        NewtonBodyGetMatrix(wheel_.newtonBody, m0.arrayof.ptr);
        NewtonBodyGetMatrix(master_.newtonBody, m1.arrayof.ptr);

        // Body pivots in Newton world coords. For the wheel it is its origin:
        // the wheel is built centered on the anchor node.
        const vec3 pivot0 = Vector3f(0.0f, 0.0f, 0.0f) * m0;
        const vec3 pivot1 = pivotMasterLocal_ * m1;

        // Master joint frame in Newton world coords.
        const vec3 front = m1.rotate(masterPinLocal_);
        const vec3 up = m1.rotate(masterUpLocal_);
        const vec3 right = m1.rotate(masterRightLocal_);

        // Three linear rows hold the pivot (axes = parent frame).
        addLinearRow(pivot0, pivot1, front);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, up);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, right);
        setRowStiffness(1.0f);

        // Wheel axle in world terms — the cylinder's local Y.
        const vec3 wheelPin = m0.rotate(Vector3f(0.0f, 1.0f, 0.0f));

        // Two angular rows kill the wheel-axle tilt against the master axes,
        // leaving the spin along the axle itself (front) free.
        addAngularRow(calculateAngle(wheelPin, front, up), up);
        setRowStiffness(1.0f);
        addAngularRow(calculateAngle(wheelPin, front, right), right);
        setRowStiffness(1.0f);
    }
}