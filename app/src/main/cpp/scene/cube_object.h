#ifndef CUBE_OBJECT_H
#define CUBE_OBJECT_H

#include "scene_object.h"
#include "shader.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

class CubeObject : public SceneObject {
public:
    CubeObject(AAssetManager* mgr) {
        addComponent<TransformComponent>();
        addComponent<ModelComponent>("models/cube_1x1x1.obj", mgr);
    }

    virtual ~CubeObject() override = default;

    void start() override {
        // Place cube at (0, 0, -5) -> 5 meters in front of camera (looking along -Z)
        getComponent<TransformComponent>()->transform.get()->setPosition(glm::vec3(0.0f, 0.0f, -5.0f));
        getComponent<TransformComponent>()->transform.get()->setScale(glm::vec3(1.0f, 1.0f, 1.0f));
    }

    void update() override {}

    void fixed_update() override {}

    void draw() override {
        if (shader) {
            shader->use();
        }
        (*shader).setMat4("model", getComponent<TransformComponent>()->transform.get()->getModelMatrix());
        getComponent<ModelComponent>()->model->Draw((*shader));
    }
};

#endif // CUBE_OBJECT_H
