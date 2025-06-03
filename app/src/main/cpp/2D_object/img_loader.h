#ifndef IMG_LOADER_H
#define IMG_LOADER_H

#include <GLES3/gl3.h>
#include <android/asset_manager.h>
#include <android/log.h>

// Now takes pointers to outW/outH. If either is nullptr, that dimension is ignored.
GLuint LoadTextureFromAsset(AAssetManager* mgr,
                            const char* filename,
                            int* outW = nullptr,
                            int* outH = nullptr);

#endif // IMG_LOADER_H
