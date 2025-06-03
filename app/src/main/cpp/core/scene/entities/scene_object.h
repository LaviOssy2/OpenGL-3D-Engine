#ifndef SCENE_OBJECT_H
#define SCENE_OBJECT_H

#include <memory>
#include <unordered_map>
#include <typeindex>
#include <typeinfo>
#include <string>
#include <stdexcept>

#include "components.h"
#include "shader.h"   // <-- we need Shader declared so draw(Shader&) compiles

class SceneObject {
public:
    SceneObject() = default;
    virtual ~SceneObject() = default;

    virtual void start() {}
    virtual void update() {}
    virtual void fixed_update() {}

    // Changed: draw now takes a Shader& so each derived object can set its own model matrix
    virtual void draw(Shader& /*shader*/) { }

    // Add a component of any type (using unique_ptr)
    template<typename T, typename... Args>
    void addComponent(Args&&... args) {
        auto type = std::type_index(typeid(T));
        if (components.find(type) != components.end()) {
            throw std::runtime_error("Component of this type already exists!");
        }
        components[type] = std::make_unique<T>(std::forward<Args>(args)...);
    }

    // Get a pointer to a component of type T
    template<typename T>
    T* getComponent() {
        auto type = std::type_index(typeid(T));
        auto it = components.find(type);
        if (it != components.end()) {
            return dynamic_cast<T*>(it->second.get());
        }
        return nullptr; // Not found
    }

private:
    std::unordered_map<std::type_index, std::unique_ptr<Component>> components;
};

#endif // SCENE_OBJECT_H
