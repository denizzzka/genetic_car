module physics_world.state;

import dlib.math.vector : Vector3f;
import dlib.math.quaternion : Quaternionf;

import physics_world.engine : PhysBody;
import physics_world.route : Route;

/// Версия раскладки снимка: файл со старой версией не должен читаться молча.
enum uint worldStateVersion = 1;

/// Поза и скорости тела в координатах каркаса.
struct BodySnapshot
{
    Vector3f position;
    Quaternionf orientation;
    Vector3f velocity;
    Vector3f angularVelocity;
}

/**
 * Снимок мира багги: тела, руль, маршрут и таймеры заезда.
 *
 * Балки, кабина и сетка рельефа сюда не входят: они выводятся из мастера,
 * руля и конфига поверхности. Каркас тоже отдельный артефакт сохранения.
 */
struct WorldSnapshot
{
    uint formatVersion = worldStateVersion;

    BodySnapshot master;
    bool hasSteer;
    BodySnapshot steer;
    BodySnapshot[] wheels;

    float steerTargetYaw;
    bool autoSteer;
    double stallTime;
    double forwardTravel;
    double time;
    float lastThrottle;

    bool hasTerrain;
    Route route;
}

/// Снять тело в снимок: поза, скорости.
BodySnapshot bodySnapshot(PhysBody body)
{
    BodySnapshot s;
    s.position = body.worldPosition;
    s.orientation = body.worldRotation;
    s.velocity = body.velocity;
    s.angularVelocity = body.angularVelocity;
    return s;
}

/// Вернуть телу позу и скорости из снимка.
void restoreBody(PhysBody body, const BodySnapshot s)
{
    body.worldTransform(s.position, s.orientation);
    body.velocity = s.velocity;
    body.angularVelocity = s.angularVelocity;
    body.syncPose();
}
