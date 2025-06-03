#include "model.h"

Model::Model(const std::string& path, AAssetManager* mgr) {
    std::vector<RawMesh> rawMeshes;
    if (!loadOBJ(path, mgr, rawMeshes)) {
        __android_log_print(ANDROID_LOG_ERROR, "Model", "loadOBJ %s failed", path.c_str());
        return;
    }

    for (size_t meshIndex = 0; meshIndex < rawMeshes.size(); ++meshIndex) {
        const auto& rm = rawMeshes[meshIndex];
        __android_log_print(ANDROID_LOG_INFO, "Model", "Mesh %zu: %zu vertices, %zu indices",
                            meshIndex, rm.vertices.size(), rm.indices.size());

        meshes.emplace_back(rm.vertices, rm.indices);
    }
}

void Model::Draw(Shader& shader) {
    shader.use();
    for (auto& mesh : meshes) {
        mesh.Draw(shader);
    }
}

