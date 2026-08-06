#ifndef SPOT_LIGHT_H
#define SPOT_LIGHT_H

#include "light.h"

class SpotLight : public Light {
public:
    glm::vec3 direction;
    float cutOff, outerCutOff;
    SpotLight(const glm::vec3& pos,
              const glm::vec3& dir,
              float cut,
              float outer,
              const glm::vec3& col);
    void apply(Shader& shader, int index) const override;
    std::string uniformPrefix(int index) const override;
};

#endif // SPOT_LIGHT_H