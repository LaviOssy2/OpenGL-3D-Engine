#ifndef CAMERA_H
#define CAMERA_H

#include "globals.h"            // brings in extern shader, surfaceW, etc.
#include "scene_object.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// A simple Camera that just has a TransformComponent. 
// In update(), you can rotate/translate it however you like.
// When drawing, the engine will grab camera->Transform to build the view matrix.
class Camera : public SceneObject {
public:
    Camera(bool applyDefaultStartTransform = true) : applyDefaultStartTransform(applyDefaultStartTransform) {
        // Add a TransformComponent so this SceneObject has a position/orientation/scale
        addComponent<TransformComponent>();

        getComponent<TransformComponent>()->transform.get()->setPosition({ 0.0f, 0.0f, 0.0f });
    }

    virtual ~Camera() override = default;

    // Called once at start
    void start() override {
        if (applyDefaultStartTransform) {
            getComponent<TransformComponent>()->transform.get()->setPosition({ 0.0f, 1.0f, 5.0f });
            getComponent<TransformComponent>()->transform.get()->translate( {0.0f, 10.0f, 15.0f} );
            getComponent<TransformComponent>()->transform.get()->rotateX(-30.f);
        } else {
            getComponent<TransformComponent>()->transform.get()->setPosition({ 0.0f, 0.0f, 0.0f });
            getComponent<TransformComponent>()->transform.get()->setRotationEuler(0.0f, 0.0f, 0.0f);
        }
    }

    // Each frame, you might allow user input to rotate the camera.
    void update() override {
        // Example: slowly spin the camera around the Y axis
        // (For a free‐look camera, you'd read input and pitch/yaw accordingly)
        //getComponent<TransformComponent>()->transform.get()->rotateY(0.5f);
        
        // You could also translate forward/backward, strafe, etc.
        // e.g. to move forward along the camera’s “forward” vector:
        // glm::vec3 forwardDir = getComponent<TransformComponent>()->transform.get()->forward();
        // getComponent<TransformComponent>()->transform.get()->translate(forwardDir * deltaTime * speed);
    }

    void draw() override {
        
        glm::mat4 V = getViewMatrix();
        glm::mat4 P = getProjectionMatrix();
        (*shader).use();
        (*shader).setMat4("view",       V);
        (*shader).setMat4("projection", P);
    }

    // Cameras typically don’t draw a mesh themselves, so no need to override draw().
    // We just supply accessors for the view matrix below:

    /// Returns the view matrix = inverse( camera’s world model matrix ).
    glm::mat4 getViewMatrix() const {
        const glm::mat4 M_cam = getComponent<TransformComponent>()->transform.get()->getModelMatrix();
        return glm::inverse(M_cam);
    }

    /// You can also expose helper to set projection externally (or leave it to engine):
    void setPerspective(float fovDeg, float aspect, float zNear, float zFar) {
        projection = glm::perspective(glm::radians(fovDeg), aspect, zNear, zFar);
    }

    glm::mat4 getProjectionMatrix() const {
        return projection;
    }

private:
    bool applyDefaultStartTransform = true;
    // Store a projection; the engine can set this once on resize, for example.
    glm::mat4 projection = glm::mat4(1.0f);
};

#endif // CAMERA_H
