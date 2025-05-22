#ifndef OBJ_LOADER_H
#define OBJ_LOADER_H
#include <android/log.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <string>
#include <android/asset_manager.h>
#include <glm/glm.hpp>
#include "vertex.h"

struct RawMesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
};

// load .obj from AssetManager into a flat
// vertex array + index list
bool loadOBJ(const std::string& path,
             AAssetManager* mgr,
             std::vector<RawMesh>& outMeshes);
             
#endif // OBJ_LOADER_H