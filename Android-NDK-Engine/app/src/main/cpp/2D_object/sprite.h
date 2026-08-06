#ifndef SPRITE_H
#define SPRITE_H

#include <vector>
#include <string>
#include <android/asset_manager.h>
#include "image.h"

class Sprite {
public:
    Sprite(const std::string& path, AAssetManager* mgr);
    void draw(Shader& shader);

private:
    std::vector<Image> sprites;  // like meshes in Model
};

#endif // SPRITE_H
