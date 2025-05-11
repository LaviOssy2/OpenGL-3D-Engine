#ifndef MESH_H
#define MESH_H

#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Shader.h"

#include <vector>
using namespace std;

struct Vertex {
    glm::vec3 Position;
};

class Mesh {
public:
    vector<Vertex>       vertices;
    vector<unsigned int> indices;
    GLuint VAO, VBO, EBO;

    Mesh(const vector<glm::vec3>& verts, const vector<unsigned int>& inds) {
        // Build Vertex list
        for (auto& v : verts) {
            Vertex vert{};
            vert.Position = v;
            vertices.push_back(vert);
        }
        indices = inds;
        setupMesh();
    }

    void Draw(Shader& shader) {
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(indices.size()),
                       GL_UNSIGNED_INT,
                       0);
        glBindVertexArray(0);
    }

private:
    void setupMesh() {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER,
                     vertices.size() * sizeof(Vertex),
                     vertices.data(),
                     GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     indices.size() * sizeof(unsigned int),
                     indices.data(),
                     GL_STATIC_DRAW);

        // position attribute
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,
                              3,
                              GL_FLOAT,
                              GL_FALSE,
                              sizeof(Vertex),
                              (void*)offsetof(Vertex, Position));

        glBindVertexArray(0);
    }
};

#endif // MESH_H