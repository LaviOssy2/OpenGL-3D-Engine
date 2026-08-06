// ObjectManager.h  (formerly OBJECT_MANAGER_H)
#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

#include "scene_object.h"
#include "camera.h"          // add this line

#include "shader.h"
#include <unordered_map>
#include <memory>
#include <string>

class ObjectManager {
public:
    // add a new object under a given name
    void addObject(const std::string& name, std::unique_ptr<SceneObject> obj) {
        objects.emplace(name, std::move(obj));
    }

    // retrieve by name (or nullptr if not found)
    SceneObject* getObject(const std::string& name) const {
        auto it = objects.find(name);
        return it != objects.end() ? it->second.get() : nullptr;
    }

    // life-cycle dispatchers
    void startAll() {
        for (auto& [_, obj] : objects) obj->start();
    }

    void updateAll() {
        for (auto& [_, obj] : objects) obj->update();
    }

    void fixedUpdateAll() {
        for (auto& [_, obj] : objects) obj->fixed_update();
    }

    void drawAll() {
        SceneObject* cam = getObject("Camera");
        if (cam) cam->draw();

        for (auto& [name, obj] : objects) {
            if (name != "Camera") {
                obj->draw();
            }
        }
    }

    void resize() {
        float aspect = surfaceW / (float)surfaceH;
        ((Camera*)getObject("Camera"))->setPerspective(45.0f, aspect, 0.1f, 100.0f);
    }

private:
    std::unordered_map<std::string, std::unique_ptr<SceneObject>> objects;
};

#endif // OBJECT_MANAGER_H
