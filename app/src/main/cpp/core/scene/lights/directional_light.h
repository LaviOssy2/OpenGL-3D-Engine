#ifndef DIRECTIONAL_LIGHT_H
#define DIRECTIONAL_LIGHT_H

#include "light.h"

class DirectionalLight : public Light {
public:
    glm::vec3 direction;
    DirectionalLight(const glm::vec3& dir, const glm::vec3& col);
    void apply(Shader& shader, int index = 0) const override;
    std::string uniformPrefix(int index) const override;
};

#endif // DIRECTIONAL_LIGHT_H