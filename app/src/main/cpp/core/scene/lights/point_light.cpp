#include "point_light.h"
#include "shader.h"

PointLight::PointLight(const glm::vec3& pos, const glm::vec3& col)
    : Light(pos, col) {}

void PointLight::apply(Shader& shader, int index) const {
    std::string p = uniformPrefix(index);
    shader.setVec3((p + ".position").c_str(), position);
    shader.setVec3((p + ".color").c_str(), color);
    shader.setFloat((p + ".constant").c_str(), 1.0f);
    shader.setFloat((p + ".linear").c_str(), 0.09f);
    shader.setFloat((p + ".quadratic").c_str(), 0.032f);
}

std::string PointLight::uniformPrefix(int index) const {
    return "pointLights[" + std::to_string(index) + "]";
}