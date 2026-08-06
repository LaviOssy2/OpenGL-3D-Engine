// Input.cpp
#include "touch_input.h"

// Storage for touch data (supports e.g. up to 10 fingers)
std::vector<std::optional<Input::TouchInfo>> Input::touchPoints(10);

void Input::UpdateTouch(int pointerIndex, int pointerId, float x, float y, int fingerCount, int action) {
    if (pointerIndex >= touchPoints.size()) return;

    if (action == 1 /*ACTION_UP*/ || action == 6 /*ACTION_POINTER_UP*/) {
        touchPoints[pointerIndex] = std::nullopt;  // Finger lifted
    } else {
        touchPoints[pointerIndex] = TouchInfo{pointerId, glm::vec2(x, y)};
    }
}

std::optional<Input::TouchInfo> Input::GetTouch(int index) {
    if (index < 0 || index >= touchPoints.size()) return std::nullopt;
    return touchPoints[index];
}
