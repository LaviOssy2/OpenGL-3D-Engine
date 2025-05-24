#ifndef COMPONENTS_H
#define COMPONENTS_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <memory>
#include "model.h"
#include "light.h"
#include "directional_light.h"
#include "point_light.h"
#include "spot_light.h"

struct Component {
    virtual ~Component() = default; // Virtual destructor
};

struct TransformComponent : Component {
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 scale;
};

struct ModelComponent : Component {
    std::unique_ptr<Model> model; // Pointer to the Mesh object
    ModelComponent(const std::string& path, AAssetManager* mgr) : model(std::make_unique<Model>(path, mgr)) {}
    ~ModelComponent() override = default;
};

struct LightComponent : Component {
    std::unique_ptr<Light> light;

    LightComponent(std::unique_ptr<Light> lightPtr)
        : light(std::move(lightPtr)) {}

};

#endif // COMPONENTS_H