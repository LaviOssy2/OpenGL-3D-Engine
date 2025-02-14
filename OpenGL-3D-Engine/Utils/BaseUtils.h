#ifndef BaseUtils_H
#define BaseUtils_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <random>
#include <algorithm>
#include <numeric> // for std::iota
#include <limits>
#include <set>
#include <map>


using namespace std;
using namespace glm;


// Define an equality function for glm::vec3 for comparison in unordered_map
struct Vec3Equal {
    bool operator()(const glm::vec3& a, const glm::vec3& b) const {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};

struct Vec3Hash {
    size_t operator()(const glm::vec3& v) const {
        return std::hash<float>()(v.x) ^ std::hash<float>()(v.y) ^ std::hash<float>()(v.z);
    }
};

#endif