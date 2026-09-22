/**
 * Колесо каркаса: ось вдоль «своей» балки узла (диск всегда перпендикулярен
 * балке, независимо от её ориентации), и револьте-шарнир, которым колесо
 * приварено к мастер-каркасу одним концом.
 *
 * Модуль намеренно листовой (не импортирует car.d): шарнир работает через
 * базовый `NewtonRigidBody`, а направление оси выводится чисто из геометрии
 * кадра, поэтому колёсная физика не зависит от сборки заезда.
 */
module physics_world.wheel;

import std.math;

import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.quaternion;

import dagon.ext.newton;

/// Радиус колеса по умолчанию, м. Значение, которое получит якорь `Anchor`,
/// если ген `wheelRadius` не задан (ручные потоки без него). Генетический
/// радиус каждого колеса масштабируется относительно этого базового:
/// внутренний радиус и ширина покрышки растут/сжимаются пропорционально.
enum float defaultWheelRadius = 0.3f;

/// Ось вращения колеса по одной балке: направление балки, приведённое к
/// единичному. Диск колеса встаёт перпендикулярно балке. Ничего не знает о
/// каркасе и мире; фолбэков нет.
vec3 wheelAxle(const vec3 beamDir)
{
    return beamDir / beamDir.length;
}

/// Своя ступица: колесо и балка делят узел якоря — ось легитимно проходит
/// через колесо, контакт с ним не считается провалом.
bool isOwnWheel(size_t wheelNode, size_t beamA, size_t beamB)
{
    return wheelNode == beamA || wheelNode == beamB;
}

/**
 * Ось колеса, закреплённая одним концом: револьте-шарнир на пользовательском
 * шарнире Newton. В отличие от BallConstraint (шаровой шарнир — точка пивота
 * зафиксирована, но все три поворота свободны, колесо болтается) здесь
 * свободен только спин вокруг оси колеса, а два перпендикулярных оси поворота
 * жёстко заблокированы — колесо катится по неподвижной оси.
 *
 * Ось шарнира — локальный Y колеса (ось цилиндра), куда бы она ни смотрела:
 * направление задаётся разворотом колеса при сборке (`wheelAxle`), шарнир
 * этого направления не знает.
 *
 * Построение повторяет опорный hinge Newton (`dCustomHinge::SubmitConstraints`):
 * три линейных ряда держат точку пивота, а два угловых ряда вокруг поперечных
 * осей РОДИТЕЛЯ гасят наклон оси колеса. Угол берётся `calculateAngle`
 * (проекция оси колеса на плоскость и atan2), как в движке, — через
 * относительный кватернион считать нельзя (знак/фрейм уезжают).
 */
final class WheelAxleJoint : NewtonUserConstraint
{
    private NewtonRigidBody wheel_;
    private NewtonRigidBody master_;
    /// Пивот шарнира в локальных координатах мастер-каркаса (Newton-мир).
    private vec3 pivotMasterLocal_;
    /// Ось шарнира и две поперечины в локальных координатах мастера.
    private vec3 masterPinLocal_;
    private vec3 masterUpLocal_;
    private vec3 masterRightLocal_;

    this(NewtonRigidBody wheel, NewtonRigidBody master, vec3 pivotMasterLocal)
    {
        super(master.world, wheel, master, 6);
        wheel_ = wheel;
        master_ = master;
        pivotMasterLocal_ = pivotMasterLocal;

        // Фрейм шарнира мастера: ось = текущая мировая ось колеса, выраженная
        // в локальных координатах мастера. На сборке мастер без поворота, но
        // считаем честно — на случай размещённого каркаса.
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

    /// Единичный вектор или нуль — ось всегда задана, но бережёмся деления на 0.
    private static vec3 unit(vec3 v)
    {
        const float l = v.length;
        return l < 1e-5f ? Vector3f(0.0f, 0.0f, 0.0f) : v / l;
    }

    /// Знаковый угол между `dir` и `cosDir` в плоскости с нормалью `sinDir`
    /// (порт `dCustomJoint::CalculateAngle`).
    private static float calculateAngle(vec3 dir, vec3 cosDir, vec3 sinDir)
    {
        const vec3 projectDir = dir - sinDir * dot(dir, sinDir);
        const float cosAngle = dot(projectDir, cosDir);
        const float sinAngle = dot(sinDir, cross(projectDir, cosDir));
        return atan2(sinAngle, cosAngle);
    }

    override void submit(float timestep, int threadIndex)
    {
        // Актуальные матрицы тел на шаге решения (не кэш обёртки — он отстаёт).
        Matrix4x4f m0, m1;
        NewtonBodyGetMatrix(wheel_.newtonBody, m0.arrayof.ptr);
        NewtonBodyGetMatrix(master_.newtonBody, m1.arrayof.ptr);

        // Пивоты тел в мировых координатах Newton. У колеса это его начало:
        // колесо построено центром в узле якоря.
        const vec3 pivot0 = Vector3f(0.0f, 0.0f, 0.0f) * m0;
        const vec3 pivot1 = pivotMasterLocal_ * m1;

        // Фрейм шарнира мастера в мировых координатах Newton.
        const vec3 front = m1.rotate(masterPinLocal_);
        const vec3 up = m1.rotate(masterUpLocal_);
        const vec3 right = m1.rotate(masterRightLocal_);

        // Три линейных ряда держат точку пивота (оси — фрейм родителя).
        addLinearRow(pivot0, pivot1, front);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, up);
        setRowStiffness(1.0f);
        addLinearRow(pivot0, pivot1, right);
        setRowStiffness(1.0f);

        // Ось колеса в мире — локальный Y цилиндра.
        const vec3 wheelPin = m0.rotate(Vector3f(0.0f, 1.0f, 0.0f));

        // Два угловых ряда гасят наклон оси колеса относительно осей мастера,
        // оставляя свободным спин вдоль самой оси (front).
        addAngularRow(calculateAngle(wheelPin, front, up), up);
        setRowStiffness(1.0f);
        addAngularRow(calculateAngle(wheelPin, front, right), right);
        setRowStiffness(1.0f);
    }
}