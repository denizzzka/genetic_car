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
import std.algorithm : clamp;

import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;

import dagon.ext.newton;

/// Default wheel radius, m. Value the `Anchor` gets when the `wheelRadius`
/// gene is absent (hand-built frames without it). Each wheel's genetic radius
/// scales relative to this base: tire inner radius and width grow/shrink
/// proportionally.
enum float defaultWheelRadius = 0.3f;

/// Полуугол поворота рулевой балки, рад (45°).
enum float steerLimitRad = 0.25f * PI;

/// Чувствительность руля: рад угла на рад отклонения от курса.
enum float steerGain = 1.0f;

/// Максимальная коррекция руля за шаг: на ненагруженной балке правка ряда
/// подпитывает своё измерение — без капа балка раскручивается в воронку.
enum float steerErrorCapRad = 0.5f;

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

/// Unit vector or zero — the axle is always defined, but guard the /0.
private vec3 unit(vec3 v)
{
    const float l = v.length;
    return l < 1e-5f ? Vector3f(0.0f, 0.0f, 0.0f) : v / l;
}

/// Signed angle between `dir` and `cosDir` in the plane with normal `sinDir`
/// (port of `dCustomJoint::CalculateAngle`).
private float calculateAngle(vec3 dir, vec3 cosDir, vec3 sinDir)
{
    const vec3 projectDir = dir - sinDir * dot(dir, sinDir);
    const float cosAngle = dot(projectDir, cosDir);
    const float sinAngle = dot(sinDir, cross(projectDir, cosDir));
    return atan2(sinAngle, cosAngle);
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
    private vec3 unit(vec3 v)
    {
        const float l = v.length;
        return l < 1e-5f ? Vector3f(0.0f, 0.0f, 0.0f) : v / l;
    }

    /// Signed angle between `dir` and `cosDir` in the plane with normal `sinDir`
    /// (port of `dCustomJoint::CalculateAngle`).
    private float calculateAngle(vec3 dir, vec3 cosDir, vec3 sinDir)
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

/**
 * Motorized hinge for the steering arm: pin joint at the pivot, tilt/roll
 * locked to the parent's vertical, one angular row around it driving the yaw
 * toward `targetYaw` relative to the parent. The per-step error cap keeps an
 * unloaded arm from spiraling (its own update feeds the measurement). Beyond
 * the range a stop row pulls the arm back to the limit. Angular locks are
 * compliant, so the arm settles within ~0.1 rad of `steerLimitRad`.
 *
 * The arm is not aligned to the parent by its own axes: only a marker vector
 * (the arm's build-time vertical in its local frame) is kept on the parent's
 * vertical, so the free DOF is exactly the yaw around it.
 */
final class SteerJoint : NewtonUserConstraint
{
    private NewtonRigidBody steer_;
    private NewtonRigidBody master_;
    /// Pivot of the joint in each body's local coordinates (Newton world).
    private vec3 pivotSteerLocal_;
    private vec3 pivotMasterLocal_;
    /// Parent's vertical axis in its local coords.
    private vec3 masterUpLocal_;
    /// The arm's vertical marker in its local coords (build-time vertical).
    private vec3 armUpLocal_;

    /// Commanded yaw of the arm relative to the parent, rad. Clamped to
    /// `steerLimitRad` in `submit`.
    float targetYaw;

    this(NewtonRigidBody steer, NewtonRigidBody master, vec3 pivotSteerLocal,
        vec3 pivotMasterLocal, vec3 armUpLocal)
    {
        super(master.world, steer, master, 8);
        steer_ = steer;
        master_ = master;
        pivotSteerLocal_ = pivotSteerLocal;
        pivotMasterLocal_ = pivotMasterLocal;
        armUpLocal_ = armUpLocal;
        masterUpLocal_ = Vector3f(0.0f, 1.0f, 0.0f);
    }

    /// Current yaw of the arm relative to the parent (for tests and the viewer).
    float yawNow() @property
    {
        Matrix4x4f m0, m1;
        NewtonBodyGetMatrix(steer_.newtonBody, m0.arrayof.ptr);
        NewtonBodyGetMatrix(master_.newtonBody, m1.arrayof.ptr);
        const vec3 up = m1.rotate(masterUpLocal_);
        const vec3 front = m1.rotate(Vector3f(0.0f, 0.0f, 1.0f));
        const vec3 steerFront = m0.rotate(Vector3f(0.0f, 0.0f, 1.0f));
        return calculateAngle(steerFront, front, up);
    }

    override void submit(float timestep, int threadIndex)
    {
        // Live body matrices at solve time (not the wrapper cache — it lags).
        Matrix4x4f m0, m1;
        NewtonBodyGetMatrix(steer_.newtonBody, m0.arrayof.ptr);
        NewtonBodyGetMatrix(master_.newtonBody, m1.arrayof.ptr);

        const vec3 pivot0 = pivotSteerLocal_ * m0;
        const vec3 pivot1 = pivotMasterLocal_ * m1;

        // Parent joint frame in Newton world coords.
        const vec3 up = m1.rotate(masterUpLocal_);
        const vec3 front = m1.rotate(Vector3f(0.0f, 0.0f, 1.0f));
        const vec3 right = m1.rotate(Vector3f(1.0f, 0.0f, 0.0f));

        // Three linear rows hold the pivot (axes = parent frame).
        addLinearRow(pivot0, pivot1, front);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, up);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, right);
        setRowStiffness(1.0f);

        // Two angular rows keep the arm's vertical marker on the parent's vertical,
        // leaving only the yaw around it free.
        const vec3 armUp = m0.rotate(armUpLocal_);
        addAngularRow(calculateAngle(armUp, up, front), front);
        setRowStiffness(1.0f);
        addAngularRow(calculateAngle(armUp, up, right), right);
        setRowStiffness(1.0f);

const vec3 steerFront = m0.rotate(Vector3f(0.0f, 0.0f, 1.0f));
        const float yaw = calculateAngle(steerFront, front, up);
        // Вне предела — один жёсткий ряд возвращает к границе (второй ряд на
        // этой оси не должен соседствовать с приводом: сингулярная матрица).
        if (yaw < -steerLimitRad)
        {
            addAngularRow(yaw + steerLimitRad, up);
            setRowStiffness(1.0f);
        }
        else if (yaw > steerLimitRad)
        {
            addAngularRow(yaw - steerLimitRad, up);
            setRowStiffness(1.0f);
        }
        else
        {
            // Ряд движет yaw к цели; кап держит на ненагруженной балке, где
            // правка ряда подпитывает своё же измерение и балка раскручивается.
            const float target = clamp(targetYaw, -steerLimitRad, steerLimitRad);
            const float err = clamp(yaw - target,
                -steerErrorCapRad, steerErrorCapRad);
            addAngularRow(err, up);
            setRowStiffness(0.5f);
        }
    }
}