#ifndef JOYSTICK_H
#define JOYSTICK_H

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
        addComponent<SpriteComponent>("images/joystick.png", mgr);
    }

    /// Call once per frame, passing in the current screen width (in pixels).
    void update(int screenW) {
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
            if (p.x <= (screenW * 0.5f)) {
                // This is the first frame of a left‐side touch → “place” the background exactly once:
                assignedFingerId = opt->pointerId;
                touched = true;
                getComponent<TransformComponent>()->transform->setPosition(glm::vec3(p, 0.0f));
                break;
            }
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
    static constexpr int maxTouchPoints = 10;
    int  assignedFingerId;   // –1 if no finger currently driving this stick
    bool touched;
};

#endif // JOYSTICK_H
