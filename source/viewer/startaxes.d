module viewer.startaxes;

import dlib.math.vector;

import dagon;

import frame.frame : origin, frameForward = forward, frameRight = right, frameUp = up;
import physics_world : dropHeight, TerrainSurface;
import viewer.scene : carToScenePos;

/// Отступ вправо по `right` от линии старта и подъём над её высотой, м.
enum float axesSideOffset = 3.0f;
enum float axesLift = 0.5f;

/// Радиус ствола стрелки, радиус и длина её наконечника, м.
enum float axesShaftRadius = 0.05f;
enum float axesHeadRadius = 0.16f;
enum float axesHeadLength = 0.45f;

/**
 * Оси каркаса у линии старта: вперёд — зелёная стрелка, вверх — синяя.
 * Часть сцены наравне с рельефом: машина от осей уезжает, а направление
 * каркаса на старте остаётся видно всегда.
 */
void buildStartAxes(Scene scene, TerrainSurface terrain)
{
    enum float forwardLength = 3.0f;
    enum float upLength = 2.0f;
    enum uint slices = 8;

    auto shaft = New!ShapeCylinder(1.0f, 1.0f, slices, scene.assetManager);
    auto head = New!ShapeCone(1.0f, 1.0f, slices, scene.assetManager);

    // Без текстуры: dagon берёт diffuse из baseColorFactor, а слабое свечение
    // emissionFactor выделяет оси на фоне рельефа.
    auto matForward = scene.addMaterial();
    matForward.baseColorFactor = Color4f(0.10f, 0.80f, 0.20f, 1.0f);
    matForward.emissionFactor = Color4f(0.05f, 0.40f, 0.10f, 1.0f);
    matForward.roughnessFactor = 0.4f;

    auto matUp = scene.addMaterial();
    matUp.baseColorFactor = Color4f(0.15f, 0.35f, 0.95f, 1.0f);
    matUp.emissionFactor = Color4f(0.05f, 0.12f, 0.45f, 1.0f);
    matUp.roughnessFactor = 0.4f;

    // Низ осей — на полметра выше раскладки старта: справа от машины
    // (по направлению «право» каркаса) оси показывают ориентацию старта.
    const vec3 ground = origin + frameRight * (-axesSideOffset);
    const vec3 base = ground
        + frameUp * (terrain.heightAt(ground) + dropHeight + axesLift);

    addArrow(scene, shaft, head, matForward, base, frameForward, forwardLength);
    addArrow(scene, shaft, head, matUp, base, frameUp, upLength);
}

/// Стрелка вдоль `dir` из точки `base` в ту же точку в базисе сцены: ствол
/// цилиндром, наконечник конусом остриём в конце.
private void addArrow(Scene scene, Mesh shaft, Mesh head, Material mat,
    const vec3 base, const vec3 dir, const float length)
{
    const vec3 neck = base + dir * (length - axesHeadLength);
    const vec3 tip = base + dir * length;

    auto eShaft = scene.addEntity();
    eShaft.drawable = shaft;
    eShaft.material = mat;
    placeAlong(eShaft, base, neck, axesShaftRadius);

    auto eHead = scene.addEntity();
    eHead.drawable = head;
    eHead.material = mat;
    placeAlong(eHead, neck, tip, axesHeadRadius);
}

/// Ставит меш (ось — локальный +Y) вдоль отрезка a→b: середина — позиция,
/// длина — масштаб по Y, радиус — по X и Z.
private void placeAlong(Entity e, const vec3 a, const vec3 b, const float radius)
{
    const vec3 dir = b - a;
    e.position = carToScenePos((a + b) * 0.5f);
    e.rotation = rotationBetween(Vector3f(0, 1, 0), carToScenePos(dir).normalized);
    e.scaling = Vector3f(radius, dir.length, radius);
}