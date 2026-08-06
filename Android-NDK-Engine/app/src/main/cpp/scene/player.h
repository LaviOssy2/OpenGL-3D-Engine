#ifndef PLAYER_H
#define PLAYER_H

#include "scene_object.h"
#include "shader.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "left_front_stick.h"

class Player : public SceneObject {
public:
    // Pass in AssetManager and optional stick pointer for movement
    Player(AAssetManager* mgr, LeftFrontStick* stick = nullptr)
        : m_stick(stick)
    {
        // 1) Add a TransformComponent and initialize default values
        addComponent<TransformComponent>();

        // 2) Add a ModelComponent (load the mesh)
        addComponent<ModelComponent>("models/2meshexp.obj", mgr);
    }

    virtual ~Player() override = default;

    void setStick(LeftFrontStick* stick) { m_stick = stick; }

    // (Optional) If you have logic you need to run once:
    void start() override {
        getComponent<TransformComponent>()->transform.get()->setPosition(glm::vec3(0.f, 4.f, 0.f));
    }

    // Called every frame by the engine
    void update() override {
        if (m_stick) {
            glm::vec2 input = m_stick->getInputValue();
            if (glm::length(input) > 0.001f) {
                float moveSpeed = 0.15f;
                // input.x: left/right, input.y: forward/backward
                glm::vec3 moveDir(input.x * moveSpeed, 0.0f, input.y * moveSpeed);
                getComponent<TransformComponent>()->transform->translate(moveDir);
            }
        }
    }

    void fixed_update() override {
        // physics or fixed‐timestep logic (if needed)
    }

    // draw now takes a Shader&, so it can set its own model matrix before drawing its mesh:
    void draw() override {
        if (shader) {
            shader->use();
        }
        // 2) Upload to the shader
        (*shader).setMat4("model", getComponent<TransformComponent>()->transform.get()->getModelMatrix());

        // 3) Draw the mesh
        getComponent<ModelComponent>()->model->Draw((*shader));
    }

private:
    LeftFrontStick* m_stick;
};

#endif // PLAYER_H

