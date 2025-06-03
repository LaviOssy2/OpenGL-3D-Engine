#ifndef MESH_H
#define MESH_H

#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <vector>
#include "vertex.h"
#include "shader.h"

class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    GLuint VAO, VBO, EBO;

    // Constructor using full vertex struct
    Mesh(const std::vector<Vertex>& verts, const std::vector<unsigned int>& inds);

    void Draw(Shader& shader);

private:
    void setupMesh();
};

#endif // MESH_H
