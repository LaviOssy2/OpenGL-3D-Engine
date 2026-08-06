#ifndef POINT_LIGHT_H
#define POINT_LIGHT_H

#include "light.h"

class PointLight : public Light {
public:
    PointLight(const glm::vec3& pos, const glm::vec3& col);
    void apply(Shader& shader, int index) const override;
    std::string uniformPrefix(int index) const override;
};

#endif // POINT_LIGHT_H