#ifndef COMPONENTS_H
#define COMPONENTS_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <memory>
#include "model.h"
#include "light.h"
#include "transform.h"
#include "directional_light.h"
#include "point_light.h"
#include "spot_light.h"
#include "sprite.h"  // Make sure this includes Image and loader logic


struct Component {
    virtual ~Component() = default; // Virtual destructor
};

struct TransformComponent : Component {
    std::unique_ptr<Transform> transform; // Pointer to the Mesh object
    TransformComponent() : transform(std::make_unique<Transform>()) {}
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

struct SpriteComponent : Component {
    std::unique_ptr<Sprite> sprite;

    SpriteComponent(const char* path, AAssetManager* mgr)
        : sprite(std::make_unique<Sprite>(path, mgr)) {}
    ~SpriteComponent() override = default;
};



#endif // COMPONENTS_H