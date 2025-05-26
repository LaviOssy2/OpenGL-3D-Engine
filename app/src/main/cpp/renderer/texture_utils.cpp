#include "texture_utils.h"
#include <vector>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

GLuint LoadTextureFromAsset(AAssetManager* mgr, const char* filename) {
    AAsset* asset = AAssetManager_open(mgr, filename, AASSET_MODE_STREAMING);
    if (!asset) {
        __android_log_print(ANDROID_LOG_ERROR, "TextureUtils", "Failed to open asset: %s", filename);
        return 0;
    }

    off_t len = AAsset_getLength(asset);
    std::vector<unsigned char> buffer(len);
    AAsset_read(asset, buffer.data(), len);
    AAsset_close(asset);

    int width, height, channels;
    unsigned char* data = stbi_load_from_memory(buffer.data(), len, &width, &height, &channels, STBI_rgb_alpha);
    if (!data) {
        __android_log_print(ANDROID_LOG_ERROR, "TextureUtils", "Failed to load image: %s", filename);
        return 0;
    }

    GLuint texID;
    glGenTextures(1, &texID);
    glBindTexture(GL_TEXTURE_2D, texID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    stbi_image_free(data);

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return texID;
}
