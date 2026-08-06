#ifndef LEFT_BACKGROUND_STICK_H
#define LEFT_BACKGROUND_STICK_H

#include <memory>
#include <android/asset_manager.h>
#include "scene_object.h"
#include "components.h"
#include "touch_input.h"
#include "shader.h"

class LeftBGStick : public SceneObject {
public:
    LeftBGStick(AAssetManager* mgr)
        : assignedFingerId(-1), touched(false)
    {
        addComponent<TransformComponent>();
        addComponent<SpriteComponent>("images/circle.png", mgr);

    }

    int getAssignedFingerId() const { return assignedFingerId; }
    bool isTouched() const { return touched; }
    glm::vec2 getPosition() const {
        return glm::vec2(getComponent<TransformComponent>()->transform->position);
    }


    /// Call once per frame
    void update() override {
        // If a finger is already assigned, check whether it’s still down:
        if (assignedFingerId != -1) {
            bool stillHere = false;
            for (int i = 0; i < maxTouchPoints; ++i) {
                auto opt = Input::GetTouch(i);
                if (!opt) continue;
                if (opt->pointerId == assignedFingerId) {
                    stillHere = true;
                    break;
                }
            }
            if (!stillHere) {
                // That finger just lifted → allow a new placement next time
                assignedFingerId = -1;
                touched = false;
            }
            return; // Do not reposition on subsequent frames
        }

        // No finger currently driving the stick background. Look for a new “Begin” on left half:
        for (int i = 0; i < maxTouchPoints; ++i) {
            auto opt = Input::GetTouch(i);
            if (!opt) continue;
            glm::vec2 p = opt->position;
            if (p.x <= (surfaceW * 0.5f)) {
                // First frame of a left-side touch → place the background:
                assignedFingerId = opt->pointerId;
                touched = true;
                getComponent<TransformComponent>()->transform->setPosition(glm::vec3(p, 0.0f));
                break;
            }
        }
    }

    void draw() override {
        if (!touched) return;

        (*joystick_shader).use();
        (*joystick_shader).setMat4("uProjection", orthoProj);

        // Model matrix from TransformComponent (scale background slightly larger e.g. 0.6)
        glm::mat4 model = getComponent<TransformComponent>()->transform->getModelMatrix();
        model = glm::scale(model, glm::vec3(0.6f));

        (*joystick_shader).setMat4("uModel", model);

        // Light & semi-transparent color tint for background
        (*joystick_shader).setVec4("uColor", glm::vec4(1.0f, 1.0f, 1.0f, 0.45f));

        // Draw the sprite
        getComponent<SpriteComponent>()->sprite->draw((*joystick_shader));
    }

private:
    static constexpr int maxTouchPoints = 10;
    int  assignedFingerId;   // -1 if no finger currently driving this stick
    bool touched;
};

#endif // LEFT_BACKGROUND_STICK_H

