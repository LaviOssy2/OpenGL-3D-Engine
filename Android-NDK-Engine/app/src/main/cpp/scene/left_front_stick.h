#ifndef LEFT_FRONT_STICK_H
#define LEFT_FRONT_STICK_H

#include <memory>
#include <android/asset_manager.h>
#include "scene_object.h"
#include "components.h"
#include "touch_input.h"
#include "shader.h"
#include "left_background_stick.h"

class LeftFrontStick : public SceneObject {
public:
    LeftFrontStick(AAssetManager* mgr, LeftBGStick* bgStick)
        : m_bgStick(bgStick), touched(false), maxRadius(100.0f)
    {
        addComponent<TransformComponent>();
        addComponent<SpriteComponent>("images/circle.png", mgr);
    }

    glm::vec2 getInputValue() const {
        return touched ? m_inputValue : glm::vec2(0.0f);
    }

    void update() override {
        if (!m_bgStick || !m_bgStick->isTouched()) {
            touched = false;
            m_inputValue = glm::vec2(0.0f);
            return;
        }

        int fingerId = m_bgStick->getAssignedFingerId();
        if (fingerId == -1) {
            touched = false;
            m_inputValue = glm::vec2(0.0f);
            return;
        }

        // Find the assigned finger's current touch position
        for (int i = 0; i < maxTouchPoints; ++i) {
            auto opt = Input::GetTouch(i);
            if (!opt) continue;
            if (opt->pointerId == fingerId) {
                touched = true;
                glm::vec2 fingerPos = opt->position;
                glm::vec2 bgPos = m_bgStick->getPosition();

                glm::vec2 delta = fingerPos - bgPos;
                float len = glm::length(delta);
                if (len > maxRadius && len > 0.0001f) {
                    delta = (delta / len) * maxRadius;
                }

                m_inputValue = delta / maxRadius;

                glm::vec2 frontPos = bgPos + delta;
                getComponent<TransformComponent>()->transform->setPosition(glm::vec3(frontPos, 0.0f));
                break;
            }
        }
    }

    void draw() override {
        if (!touched) return;

        (*joystick_shader).use();
        (*joystick_shader).setMat4("uProjection", orthoProj);

        // Model matrix from TransformComponent (scale front handle smaller e.g. 0.35)
        glm::mat4 model = getComponent<TransformComponent>()->transform->getModelMatrix();
        model = glm::scale(model, glm::vec3(0.35f));

        (*joystick_shader).setMat4("uModel", model);

        // Solid opaque dark color tint (alpha = 1.0) for the front stick handle
        (*joystick_shader).setVec4("uColor", glm::vec4(0.2f, 0.2f, 0.22f, 1.0f));

        // Draw the sprite
        getComponent<SpriteComponent>()->sprite->draw((*joystick_shader));
    }


private:
    static constexpr int maxTouchPoints = 10;
    LeftBGStick* m_bgStick;
    bool touched;
    float maxRadius;
    glm::vec2 m_inputValue{0.0f, 0.0f};
};

#endif // LEFT_FRONT_STICK_H

