#ifndef SHADER_H
#define SHADER_H

#include <GLES3/gl3.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <android/log.h>  // for __android_log_print
#include <vector>
#include <iostream>

class Shader {
public:
    GLuint ID;

    Shader(const char* vertPath,
           const char* fragPath,
           AAssetManager* mgr) {
        std::string vCode = loadAsset(vertPath, mgr);
        std::string fCode = loadAsset(fragPath, mgr);
        const char* vSrc = vCode.c_str();
        const char* fSrc = fCode.c_str();

        GLuint vert = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vert, 1, &vSrc, nullptr);
        glCompileShader(vert);
        checkCompileErrors(vert, "VERTEX");

        GLuint frag = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(frag, 1, &fSrc, nullptr);
        glCompileShader(frag);
        checkCompileErrors(frag, "FRAGMENT");

        ID = glCreateProgram();
        glAttachShader(ID, vert);
        glAttachShader(ID, frag);
        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");

        glDeleteShader(vert);
        glDeleteShader(frag);
    }

    void use() {
        glUseProgram(ID);
    }

    void setMat4(const char* name, const glm::mat4& mat) {
        GLint loc = glGetUniformLocation(ID, name);
        glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(mat));
    }

private:
    std::string loadAsset(const char* path, AAssetManager* mgr) {
        AAsset* asset = AAssetManager_open(mgr, path, AASSET_MODE_BUFFER);
        if (!asset) {
            __android_log_print(ANDROID_LOG_ERROR, "Shader", "Unable to open asset '%s'", path);
            return std::string();
        }
        size_t size = AAsset_getLength(asset);
        std::string buf(size, '\0');
        AAsset_read(asset, &buf[0], size);
        AAsset_close(asset);
        return buf;
    }

    void checkCompileErrors(GLuint object, const std::string& type) {
        GLint success;
        if (type != "PROGRAM") {
            glGetShaderiv(object, GL_COMPILE_STATUS, &success);
            if (!success) {
                char info[512];
                glGetShaderInfoLog(object, 512, nullptr, info);
                std::cerr << "Shader compile error (" << type << ": " << info << std::endl;
            }
        } else {
            glGetProgramiv(object, GL_LINK_STATUS, &success);
            if (!success) {
                char info[512];
                glGetProgramInfoLog(object, 512, nullptr, info);
                std::cerr << "Program link error: " << info << std::endl;
            }
        }
    }
};

#endif // SHADER_H