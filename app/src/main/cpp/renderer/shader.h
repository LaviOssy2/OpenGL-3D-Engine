#ifndef SHADER_H
#define SHADER_H

#include <GLES3/gl3.h>
#include <android/asset_manager.h>
#include <glm/glm.hpp>
#include <string>

class Shader {
public:
    GLuint ID;

    Shader(const char* vertPath, const char* fragPath, AAssetManager* mgr);
    void use();
    void setMat4(const char* name, const glm::mat4& M);
    void setVec3(const char* name, const glm::vec3& v);
    void setVec4(const char* name, const glm::vec4& v);
    void setVec2(const char* name, const glm::vec2& v);

    void setFloat(const char* name, float value);
    void setInt(const char* name, int value);
    void setBool(const char* name, bool value);


private:
    std::string loadAsset(const char* path, AAssetManager* mgr);
    void checkErrors(GLuint obj, const char* type);
};

#endif // SHADER_H
