#ifndef LIGHT_H
#define LIGHT_H

#include <glm/glm.hpp>
#include <string>

class Shader;

class Light {
public:
    glm::vec3 position;
    glm::vec3 color;
    Light(const glm::vec3& pos, const glm::vec3& col)
        : position(pos), color(col) {}
    virtual ~Light() = default;
    virtual void apply(Shader& shader, int index) const = 0;
    virtual std::string uniformPrefix(int index) const = 0;
};

#endif // LIGHT_H