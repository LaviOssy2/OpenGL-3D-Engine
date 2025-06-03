#ifndef IMAGE_H
#define IMAGE_H

#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include "shader.h"

class Image {
public:
    Image(GLuint tex, int w, int h);
    void draw(Shader& shader);

private:
    GLuint texID, VAO, VBO;
    float width, height;
};

#endif // IMAGE_H
