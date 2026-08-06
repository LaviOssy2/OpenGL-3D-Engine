#include "sprite.h"
#include <android/log.h>
#include "img_loader.h"

Sprite::Sprite(const std::string& path, AAssetManager* mgr) {
    int w = 0, h = 0;
    GLuint texID = LoadTextureFromAsset(mgr, path.c_str(), &w, &h);
    if (texID == 0) {
        __android_log_print(ANDROID_LOG_ERROR, "Sprite", "Failed to load image: %s", path.c_str());
        return;
    }

    __android_log_print(ANDROID_LOG_INFO, "Sprite", "Loaded image: %s (%dx%d)", path.c_str(), w, h);

    sprites.emplace_back(texID, w, h);  // this Image should set up VAO/VBO
}

void Sprite::draw(Shader& shader) {
    for (auto& sprite : sprites) {
        sprite.draw(shader);
    }
}
