#include "model.h"

Model::Model(const std::string& path, AAssetManager* mgr) {
    std::vector<Vertex> verts;
    std::vector<unsigned int> idx;

    if (!loadOBJ(path, mgr, verts, idx)) {
        __android_log_print(ANDROID_LOG_ERROR,"Model","loadOBJ %s failed", path.c_str());
        return;
    }

    for (size_t i = 0; i < verts.size(); ++i) {
        const auto& v = verts[i];
        __android_log_print(ANDROID_LOG_INFO, "VertexDebug",
            "Vertex %zu: pos=(%.2f, %.2f, %.2f) norm=(%.2f, %.2f, %.2f) uv=(%.2f, %.2f)",
            i,
            v.position.x, v.position.y, v.position.z,
            v.normal.x, v.normal.y, v.normal.z,
            v.texCoord.x, v.texCoord.y);
    }

    // Create a mesh from parsed data
    meshes.emplace_back(verts, idx);
}

void Model::Draw(Shader& shader, int w, int h) {
    shader.use();
    for (auto& mesh : meshes) {
        mesh.Draw(shader);
    }
}

