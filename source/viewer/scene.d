module viewer.scene;

import dlib.math.vector;

/// Перевод из car-координат в координаты сцены dagon: поперёк +X, вверх +Y,
/// курс +Z. Единственное место, где оси каркаса превращаются в оси сцены:
/// сущности под carRoot/galleryRoot поворот делают сами, а вот геометрия,
/// которая вешается в корень сцены (тайлы рельефа), и цель камеры приходят
/// в координатах каркаса.
vec3 carToScenePos(const vec3 carPos) @property pure nothrow @safe
{
    return Vector3f(carPos.x, carPos.z, -carPos.y);
}
