#ifndef SUNLIGHT_H
#define SUNLIGHT_H

#include "scene_object.h"

class SunLight : public SceneObject {
public:
    SunLight() {
        // Add a directional light component (sun)
        addComponent<LightComponent>(
            std::make_unique<PointLight>(glm::vec3(1.0f, 3.0f, 2.0f), glm::vec3(1.0f, 0.0f, 1.0f))

        );
    }
};

#endif // SUNLIGHT_H
