#ifndef PLAYER_H
#define PLAYER_H

#include "scene_object.h"

class Player : public SceneObject {
public:
    Player(AAssetManager* mgr) {
        addComponent<TransformComponent>();
        addComponent<ModelComponent>("models/2meshexp.obj", mgr);
    }
};

#endif // PLAYER_H
