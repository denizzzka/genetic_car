module app;

import dagon;
import viewer;

void main(string[] args)
{
    MyGame game = New!MyGame(1280, 720, false, "Genetic Car - Frame Viewer", args);
    game.run();
    Delete(game);
}