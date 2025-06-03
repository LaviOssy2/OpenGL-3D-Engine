#include "shader.h"
#include <glm/gtc/type_ptr.hpp>
#include <android/log.h>
#include <string>

Shader::Shader(const char* vertPath, const char* fragPath, AAssetManager* mgr) {
    std::string vS = loadAsset(vertPath, mgr);
    std::string fS = loadAsset(fragPath, mgr);
    const char* vs = vS.c_str();
    const char* fs = fS.c_str();

    GLuint V = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(V, 1, &vs, nullptr);
    glCompileShader(V); checkErrors(V, "VERTEX");

    GLuint F = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(F, 1, &fs, nullptr);
    glCompileShader(F); checkErrors(F, "FRAGMENT");

    ID = glCreateProgram();
    glAttachShader(ID, V);
    glAttachShader(ID, F);
    glLinkProgram(ID);
    checkErrors(ID, "PROGRAM");

    glDeleteShader(V);
    glDeleteShader(F);
}

void Shader::use() {
    glUseProgram(ID);
}

void Shader::setMat4(const char* name, const glm::mat4& M) {
    GLint loc = glGetUniformLocation(ID, name);
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(M));
}

void Shader::setVec3(const char* name, const glm::vec3& v) {
    GLint loc = glGetUniformLocation(ID, name);
    glUniform3fv(loc, 1, glm::value_ptr(v));
}

void Shader::setVec4(const char* name, const glm::vec4& v) {
    GLint loc = glGetUniformLocation(ID, name);
    glUniform4fv(loc, 1, glm::value_ptr(v));
}

void Shader::setVec2(const char* name, const glm::vec2& v) {
    GLint loc = glGetUniformLocation(ID, name);
    glUniform2fv(loc, 1, glm::value_ptr(v));
}

void Shader::setFloat(const char* name, float value) {
    GLint loc = glGetUniformLocation(ID, name);
    glUniform1f(loc, value);
}

void Shader::setInt(const char* name, int value) {
    GLint loc = glGetUniformLocation(ID, name);
    glUniform1i(loc, value);
}

void Shader::setBool(const char* name, bool value) {
    GLint loc = glGetUniformLocation(ID, name);
    glUniform1i(loc, (int)value);
}


std::string Shader::loadAsset(const char* path, AAssetManager* mgr) {
    AAsset* a = AAssetManager_open(mgr, path, AASSET_MODE_BUFFER);
    if (!a) {
        __android_log_print(ANDROID_LOG_ERROR, "Shader", "open %s failed", path);
        return "";
    }
    size_t sz = AAsset_getLength(a);
    std::string out(sz, '\0');
    AAsset_read(a, &out[0], sz);
    AAsset_close(a);
    return out;
}

void Shader::checkErrors(GLuint obj, const char* type) {
    GLint ok;
    if (std::string(type) != "PROGRAM") {
        glGetShaderiv(obj, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char buf[512];
            glGetShaderInfoLog(obj, 512, nullptr, buf);
            __android_log_print(ANDROID_LOG_ERROR, "Shader", "%s compile: %s", type, buf);
        }
    } else {
        glGetProgramiv(obj, GL_LINK_STATUS, &ok);
        if (!ok) {
            char buf[512];
            glGetProgramInfoLog(obj, 512, nullptr, buf);
            __android_log_print(ANDROID_LOG_ERROR, "Shader", "link error: %s", buf);
        }
    }
}
