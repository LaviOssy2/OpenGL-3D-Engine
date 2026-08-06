#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <memory>
#include <android/asset_manager.h>
#include "scene_object.h"
#include "components.h"
#include "touch_input.h"
#include "shader.h"

class Joystick : public SceneObject {
public:
    Joystick(AAssetManager* mgr) {
        // 1. Transform for position
        addComponent<TransformComponent>();

        // 2. Sprite image component
        addComponent<SpriteComponent>("images/joystick.png", mgr);
    }

    void update() override {
        auto touch0 = Input::GetTouch(0);
        if (touch0) {
            touched = true;
            getComponent<TransformComponent>()->transform->setPosition(glm::vec3(touch0->position, 0.0f));
        } else {
            touched = false;
        }
    }

    void draw(Shader& shader, const glm::mat4& orthoProj) {
        if (!touched) return;

        shader.use();
        shader.setMat4("uProjection", orthoProj);

        // Model matrix from TransformComponent
        glm::mat4 model = getComponent<TransformComponent>()->transform->getModelMatrix();
        model = glm::scale(model, glm::vec3(0.5f));  // Apply additional scaling

        shader.setMat4("uModel", model);

        // Draw the sprite
        getComponent<SpriteComponent>()->sprite->draw(shader);
    }

private:
    bool touched = false;
};

#endif // JOYSTICK_H
