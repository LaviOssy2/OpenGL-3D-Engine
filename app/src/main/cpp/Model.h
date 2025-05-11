#ifndef MODEL_H
#define MODEL_H

#include "Mesh.h"
#include "Shader.h"
#include <vector>
#include <glm/glm.hpp>
using namespace std;

class Model {
public:
    Mesh mesh;

    Model(const vector<glm::vec3>& verts,
          const vector<unsigned int>& inds)
        : mesh(verts, inds) {}

    void Draw(Shader& shader) {
        mesh.Draw(shader);
    }
};

#endif