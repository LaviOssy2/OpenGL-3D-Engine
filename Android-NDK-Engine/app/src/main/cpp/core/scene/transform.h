#ifndef TRANSFORM_H
#define TRANSFORM_H

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>  // for glm::translate, glm::scale
#include <glm/gtc/quaternion.hpp>        // for glm::quat, glm::angleAxis
#include <glm/gtx/quaternion.hpp>        // for glm::toMat4, glm::eulerAngles

struct Transform {
    // Public data members
    glm::vec3 position;    // Translation offset
    glm::quat rotation;    // Orientation, stored as a unit quaternion
    glm::vec3 scale;       // Non‐uniform scale

    // ------------------------------------------------------------------------
    // Constructors
    // ------------------------------------------------------------------------
    Transform()
        : position(0.0f, 0.0f, 0.0f),
          rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)), // identity quaternion
          scale(1.0f, 1.0f, 1.0f)
    {}

    explicit Transform(
        const glm::vec3& pos,
        const glm::quat& rot,
        const glm::vec3& scl
    ) : position(pos),
        rotation(glm::normalize(rot)),
        scale(scl)
    {}

    // ------------------------------------------------------------------------
    // Translation
    // ------------------------------------------------------------------------
    /// Set absolute world‐space position
    void setPosition(const glm::vec3& pos) {
        position = pos;
    }

    /// Move by a delta vector in world space
    void translate(const glm::vec3& delta) {
        position += delta;
    }

    // ------------------------------------------------------------------------
    // Rotation
    // ------------------------------------------------------------------------
    /// Set orientation via a unit quaternion (overwrites prior rotation)
    void setRotation(const glm::quat& q) {
        rotation = glm::normalize(q);
    }

    /// Set orientation via Euler angles in degrees (pitch = X, yaw = Y, roll = Z)
    void setRotationEuler(float pitchDeg, float yawDeg, float rollDeg) {
        glm::vec3 eulerRad = glm::radians(glm::vec3(pitchDeg, yawDeg, rollDeg));
        glm::quat q = glm::quat(eulerRad); // GLM constructs as quat(yaw-pitch-roll) internally
        rotation = glm::normalize(q);
    }

    /// Rotate around an arbitrary axis (in local space) by X degrees
    void rotate(const glm::vec3& axis, float angleDeg) {
        float angleRad = glm::radians(angleDeg);
        glm::quat dq = glm::angleAxis(angleRad, glm::normalize(axis));
        rotation = glm::normalize(dq * rotation);
    }

    /// Rotate around X axis by angleDeg degrees (local X)
    void rotateX(float angleDeg) {
        float angleRad = glm::radians(angleDeg);
        glm::quat dq = glm::angleAxis(angleRad, glm::vec3(1.0f, 0.0f, 0.0f));
        rotation = glm::normalize(dq * rotation);
    }

    /// Rotate around Y axis by angleDeg degrees (local Y)
    void rotateY(float angleDeg) {
        float angleRad = glm::radians(angleDeg);
        glm::quat dq = glm::angleAxis(angleRad, glm::vec3(0.0f, 1.0f, 0.0f));
        rotation = glm::normalize(dq * rotation);
    }

    /// Rotate around Z axis by angleDeg degrees (local Z)
    void rotateZ(float angleDeg) {
        float angleRad = glm::radians(angleDeg);
        glm::quat dq = glm::angleAxis(angleRad, glm::vec3(0.0f, 0.0f, 1.0f));
        rotation = glm::normalize(dq * rotation);
    }

    // ------------------------------------------------------------------------
    // Scaling
    // ------------------------------------------------------------------------
    /// Set absolute scale
    void setScale(const glm::vec3& scl) {
        scale = scl;
    }

    /// Multiply current scale by a factor (component‐wise)
    void scaleBy(const glm::vec3& scl) {
        scale *= scl;
    }

    // ------------------------------------------------------------------------
    // Model‐matrix generation
    // ------------------------------------------------------------------------
    /// Compute the 4×4 model matrix: T * R * S
    glm::mat4 getModelMatrix() const {
        // 1) Translation
        glm::mat4 T = glm::translate(glm::mat4(1.0f), position);

        // 2) Rotation (convert quaternion → mat4)
        glm::mat4 R = glm::toMat4(rotation);

        // 3) Scale
        glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);

        // Combine in order: Translate, then Rotate, then Scale
        return T * R * S;
    }

    // ------------------------------------------------------------------------
    // Direction vectors (in world space)
    // ------------------------------------------------------------------------
    /// Returns the forward (−Z) vector in world coordinates
    glm::vec3 forward() const {
        // In OpenGL, a “forward” of (0,0,−1) in model space is common
        return rotation * glm::vec3(0.0f, 0.0f, -1.0f);
    }

    /// Returns the right (+X) vector in world coordinates
    glm::vec3 right() const {
        return rotation * glm::vec3(1.0f, 0.0f, 0.0f);
    }

    /// Returns the up (+Y) vector in world coordinates
    glm::vec3 up() const {
        return rotation * glm::vec3(0.0f, 1.0f, 0.0f);
    }

    // ------------------------------------------------------------------------
    // Look‐at helper
    // ------------------------------------------------------------------------
    /// Make this transform “look at” a target point in world space, using given up direction.
    /// This writes only to rotation; position and scale remain unchanged.
    void lookAt(const glm::vec3& target, const glm::vec3& worldUp = glm::vec3(0,1,0)) {
        // 1) Compute forward vector from position → target
        glm::vec3 F = glm::normalize(target - position);
        // 2) Compute right vector
        glm::vec3 Rv = glm::normalize(glm::cross(F, worldUp));
        // 3) Compute corrected up vector
        glm::vec3 U = glm::cross(Rv, F);
        // 4) Build 3×3 rotation matrix (column‐major)
        glm::mat3 rotMat;
        rotMat[0] = Rv;       // X axis
        rotMat[1] = U;        // Y axis
        rotMat[2] = -F;       // Z axis (−forward)
        // 5) Convert to quaternion
        rotation = glm::normalize(glm::quat_cast(rotMat));
    }

    // ------------------------------------------------------------------------
    // Helpers for setting from Euler angles & retrieving Euler
    // ------------------------------------------------------------------------
    /// Set orientation from Euler angles (degrees) in Y–X–Z order (yaw, pitch, roll)
    void setYawPitchRoll(float yawDeg, float pitchDeg, float rollDeg) {
        glm::vec3 eulerRad = glm::radians(glm::vec3(pitchDeg, yawDeg, rollDeg));
        rotation = glm::normalize(glm::quat(eulerRad));
    }

    /// Get Euler angles (degrees) from current rotation (in Y–X–Z order)
    glm::vec3 getYawPitchRoll() const {
        glm::vec3 eulerRad = glm::eulerAngles(rotation);
        return glm::degrees(eulerRad);
    }
};

#endif // TRANSFORM_H
