#ifndef TEXTURE_UTILS
#define TEXTURE_UTILS

#include <GLES3/gl3.h>
#include <android/asset_manager.h>
#include <android/log.h>

GLuint LoadTextureFromAsset(AAssetManager* mgr, const char* filename);

#endif // TEXTURE_UTILS