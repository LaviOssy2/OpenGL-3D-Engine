#ifndef MODEL_H
#define MODEL_H
#include <GLES3/gl3.h>
#include <android/asset_manager.h>
#include "mesh.h"                // Include your Mesh class
#include "obj_loader.h"    // Still needed for loading geometry
#include "shader.h"

class Model {
public:
    Model(const std::string& path, AAssetManager* mgr);
    ~Model() = default;

    void Draw(Shader& shader);

private:
    std::vector<Mesh> meshes;   // Use Mesh to hold geometry
};

#endif // MODEL_H