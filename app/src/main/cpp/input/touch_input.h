#pragma once
#include <glm/glm.hpp>
#include <optional>
#include <vector>

class Input {
public:
    struct TouchInfo {
        int pointerId;
        glm::vec2 position;
    };

    // Update the state (called from JNI/nativeOnTouch)
    static void UpdateTouch(int pointerIndex, int pointerId, float x, float y, int fingerCount, int action);

    // Get touch info (or std::nullopt if no touch)
    static std::optional<TouchInfo> GetTouch(int index);

private:
    static std::vector<std::optional<TouchInfo>> touchPoints;
};