#ifndef PLAYER_H
#define PLAYER_H

#include "scene_object.h"
#include "shader.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Player : public SceneObject {
public:
    // Pass in the Android AssetManager so ModelComponent can load the .obj
    Player(AAssetManager* mgr) {
        // 1) Add a TransformComponent and initialize default values
        addComponent<TransformComponent>();

        // 2) Add a ModelComponent (load the mesh)
        addComponent<ModelComponent>("models/2meshexp.obj", mgr);
    }

    virtual ~Player() override = default;

    // (Optional) If you have logic you need to run once:
    void start() override {
        // e.g. any startup logic for Player
    }

    // Called every frame by the engine
    void update() override {
        getComponent<TransformComponent>()->transform.get()->rotateY(1.0f);
        getComponent<TransformComponent>()->transform.get()->rotateX(1.0f);

    }
    

    void fixed_update() override {
        // physics or fixed‐timestep logic (if needed)
    }

    // draw now takes a Shader&, so it can set its own model matrix before drawing its mesh:
    void draw(Shader& shader) override {
        // 2) Upload to the shader
        shader.setMat4("model", getComponent<TransformComponent>()->transform.get()->getModelMatrix());

        // 3) Draw the mesh
        getComponent<ModelComponent>()->model->Draw(shader);
    }

private:
    // (No need for a local AAssetManager* member—ModelComponent holds it internally)
};

#endif // PLAYER_H
