#include "image.h"
#include <android/log.h>

Image::Image(GLuint tex, int w, int h)
    : texID(tex), width(static_cast<float>(w)), height(static_cast<float>(h))
{
    float halfW = width * 0.5f;
    float halfH = height * 0.5f;

    // Define a centered quad (6 vertices: 2 triangles)
    float quadVertices[] = {
        // pos.x, pos.y,   u, v
        -halfW, -halfH,   0.0f, 0.0f,
         halfW, -halfH,   1.0f, 0.0f,
         halfW,  halfH,   1.0f, 1.0f,

        -halfW, -halfH,   0.0f, 0.0f,
         halfW,  halfH,   1.0f, 1.0f,
        -halfW,  halfH,   0.0f, 1.0f
    };

    // Upload quad data to GPU
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

    // layout(location = 0): vec2 aPos
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0
    );

    // layout(location = 1): vec2 aTexCoord
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float))
    );

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Image::draw(Shader& shader) {
    // Disable depth test so it draws on top
    glDisable(GL_DEPTH_TEST);

    shader.use();

    // Set uniforms
    shader.setInt("uTexture", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texID);

    // uProjection should be set externally before calling draw()
    // uPosition should also be set externally via shader.setVec2("uPosition", ...)

    // Draw the quad
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
}