#ifndef SCENE_OBJECT_H
#define SCENE_OBJECT_H

#include <memory>
#include <unordered_map>
#include <typeindex>
#include <typeinfo>
#include <string>
#include <stdexcept>

#include "components.h"

class SceneObject {
public:
    SceneObject() = default;
    ~SceneObject() = default;

    // Add a component of any type (using unique_ptr)
    template<typename T, typename... Args>
    void addComponent(Args&&... args) {
        auto type = std::type_index(typeid(T));
        if (components.find(type) != components.end()) {
            throw std::runtime_error("Component of this type already exists!");
        }
        components[type] = std::make_unique<T>(std::forward<Args>(args)...);
    }

    // Get a reference to a component of type T
    template<typename T>
    T* getComponent() {
        auto type = std::type_index(typeid(T));
        auto it = components.find(type);
        if (it != components.end()) {
            return dynamic_cast<T*>(it->second.get());
        }
        return nullptr; // Not found
    }

    // Update all components
    // void updateComponents() {
    //     for (auto& [type, comp] : components) {
    //         comp->update();
    //     }
    // }

private:
    std::unordered_map<std::type_index, std::unique_ptr<Component>> components;
};

#endif // SCENE_OBJECT_H
