#include "spot_light.h"
#include "shader.h"

SpotLight::SpotLight(const glm::vec3& pos,
                     const glm::vec3& dir,
                     float cut,
                     float outer,
                     const glm::vec3& col)
    : Light(pos, col), direction(dir), cutOff(cut), outerCutOff(outer) {}

void SpotLight::apply(Shader& shader, int index) const {
    std::string p = uniformPrefix(index);
    shader.setVec3((p + ".position").c_str(), position);
    shader.setVec3((p + ".direction").c_str(), direction);
    shader.setFloat((p + ".cutOff").c_str(), cutOff);
    shader.setFloat((p + ".outerCutOff").c_str(), outerCutOff);
    shader.setVec3((p + ".color").c_str(), color);
    shader.setFloat((p + ".constant").c_str(), 1.0f);
    shader.setFloat((p + ".linear").c_str(), 0.09f);
    shader.setFloat((p + ".quadratic").c_str(), 0.032f);
}

std::string SpotLight::uniformPrefix(int index) const {
    return "spotLights[" + std::to_string(index) + "]";
}