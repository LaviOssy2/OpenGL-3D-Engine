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


// load .obj from AssetManager into a flat
// vertex array + index list
bool loadOBJ(const std::string&    path,
             AAssetManager*        mgr,
             std::vector<Vertex>&       outVertices,
             std::vector<unsigned int>& outIndices);
             
#endif // OBJ_LOADER_H