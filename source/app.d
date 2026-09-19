module app;

import dagon;
import viewer;
import physics_world;

void main(string[] args)
{
    // Грузим libnewton.so до старта и до физических воркеров: bindbc
    // резолвит символы один раз на процесс, повторно — защита в
    // BuggyPhysics (ensureNewtonLoaded).
    ensureNewtonLoaded();

    MyGame game = New!MyGame(1280, 720, false, "Genetic Car - Frame Viewer", args);
    game.run();
    Delete(game);
}