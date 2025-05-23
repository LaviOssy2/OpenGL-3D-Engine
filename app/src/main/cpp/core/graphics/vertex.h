#ifndef VERTEX_H
#define VERTEX_H

#define MAX_BONE_INFLUENCE 4

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texCoord;

    // Optional: unused for now, but zero-initialized
    glm::vec3 Tangent = glm::vec3(0.0f);
    glm::vec3 Bitangent = glm::vec3(0.0f);
    int m_BoneIDs[MAX_BONE_INFLUENCE] = {0, 0, 0, 0};
    float m_Weights[MAX_BONE_INFLUENCE] = {0.0f, 0.0f, 0.0f, 0.0f};
};

#endif // VERTEX_H
