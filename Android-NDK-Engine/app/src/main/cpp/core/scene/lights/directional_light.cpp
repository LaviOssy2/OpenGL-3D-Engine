#include "directional_light.h"
#include "shader.h"

DirectionalLight::DirectionalLight(const glm::vec3& dir, const glm::vec3& col)
    : Light(glm::vec3(0.0f), col), direction(dir) {}

void DirectionalLight::apply(Shader& shader, int /*index*/) const {
    const auto p = "dirLight";
    shader.setVec3((p + std::string(".direction")).c_str(), direction);
    shader.setVec3((p + std::string(".color")).c_str(), color);
}

std::string DirectionalLight::uniformPrefix(int) const {
    return "dirLight";  // always a single directional light
}