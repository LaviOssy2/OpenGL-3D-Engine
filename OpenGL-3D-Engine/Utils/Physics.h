#ifndef Physics_H
#define Physics_H

#include "Mathematics.h"

struct CollisionBox {
    bool convex;
    bool precise;
    vector<vec3> orgPoints;
    vector<vec3> currentPoints;
    vector<unsigned int> inds;
    SMesh* mesh;
    vec3 min;
    vec3 max;
};

static vector<CollisionBox> AssignCollisionBoxes(std::unordered_map<Triangle, int, TriangleHash, TriangleEqual> assignments, const vector<vec3>& orgVerts, int k) {
    std::vector<int> clusterMap(k, -1); // Maps old cluster indices to new indices
    std::vector<bool> hasTriangles(k, false); // Tracks if the cluster has any triangles

    // First pass: Identify non-empty clusters
    for (const auto& assignment : assignments) {
        hasTriangles[assignment.second] = true;
    }

    // Prepare mapping and count non-empty clusters
    int nonEmptyCount = 0;
    for (int i = 0; i < k; ++i) {
        if (hasTriangles[i]) {
            clusterMap[i] = nonEmptyCount++;
        }
    }

    vector<CollisionBox> subBoxes(nonEmptyCount);
    vector<std::unordered_map<vec3, unsigned int, Vec3Hash, Vec3Equal>> vertsMaps(nonEmptyCount);

    // Initialize bounding boxes
    vector<vec3> currentMin(nonEmptyCount, vec3(FLT_MAX));
    vector<vec3> currentMax(nonEmptyCount, vec3(-FLT_MAX));

    // Process assignments
    for (const auto& assignment : assignments) {
        int newClusterIndex = clusterMap[assignment.second];
        if (newClusterIndex != -1) { // Only add to boxes if the cluster is not empty
            CollisionBox& box = subBoxes[newClusterIndex];
            Triangle t = assignment.first;

            auto processVertex = [&](int vertexIdx) {
                const vec3& vertex = orgVerts[vertexIdx];
                if (vertsMaps[newClusterIndex].find(vertex) == vertsMaps[newClusterIndex].end()) {
                    vertsMaps[newClusterIndex][vertex] = box.orgPoints.size();
                    box.orgPoints.push_back(vertex);

                    // Update bounding box
                    currentMin[newClusterIndex] = glm::min(currentMin[newClusterIndex], vertex);
                    currentMax[newClusterIndex] = glm::max(currentMax[newClusterIndex], vertex);
                }
                box.inds.push_back(vertsMaps[newClusterIndex][vertex]);
                };

            processVertex(t.i0);
            processVertex(t.i1);
            processVertex(t.i2);
        }
    }

    // Finalize bounding boxes
    for (int i = 0; i < nonEmptyCount; ++i) {
        subBoxes[i].min = currentMin[i];
        subBoxes[i].max = currentMax[i];
        subBoxes[i].currentPoints = subBoxes[i].orgPoints;
        subBoxes[i].mesh = new SMesh(create_mesh(subBoxes[i].currentPoints, subBoxes[i].inds, vec3(0.f)));

    }

    return subBoxes;
}

static void AssignOBB(CollisionBox& obb)
{
    obb.precise = false;
    obb.orgPoints = ComputeOBBCorners(obb.currentPoints);
    obb.currentPoints = obb.orgPoints;
    obb.inds = //inds;
    {
        // Front face
        0, 1, 3,
        0, 3, 2,
        // Back face
        4, 5, 7,
        4, 7, 6,
        // Left face
        0, 2, 6,
        0, 6, 4,
        // Right face
        1, 3, 7,
        1, 7, 5,
        // Top face
        2, 3, 7,
        2, 7, 6,
        // Bottom face
        0, 1, 5,
        0, 5, 4
    };
}

static void AssignNewConvexHull(CollisionBox& box)
{
    box.convex = true;
    vector<vec3> verts;
    vector<unsigned int> inds;

    vector<vec3> points;
    for (int g : box.inds)
        points.push_back(box.orgPoints[g]);


    BuildConvexHullPolyhedron(points, verts, inds);

    box.orgPoints = verts;
    box.currentPoints = verts;
    box.inds = inds;
}

static float DistanceToBoundingBox(vec3 point, CollisionBox& box)
{
    glm::vec3 closestPoint;
    glm::vec3 boxMin = box.min;
    glm::vec3 boxMax = box.max;
    closestPoint.x = std::clamp(point.x, boxMin.x, boxMax.x);
    closestPoint.y = std::clamp(point.y, boxMin.y, boxMax.y);
    closestPoint.z = std::clamp(point.z, boxMin.z, boxMax.z);
    return length(point - closestPoint);
    // Calculate the squared distance between the point and the closest point
    glm::vec3 delta = point - closestPoint;
    float distanceSquared = glm::dot(delta, delta);

    // Return the actual distance
    return std::sqrt(distanceSquared);
}

static bool DetectCollision(vector<vec3>& v1, vector<unsigned int>& i1, vector<vec3>& v2, vector<unsigned int>& i2, vec3 TO)
{
    SMesh a = create_mesh(v1, i1, vec3(0.f));
    SMesh b = create_mesh(v2, i2, TO);

    return meshes_intersect(a, b);

}

static bool DetectCollisionV(vector<vec3>& v1, vector<unsigned int>& i1, vector<vec3>& v2, vector<unsigned int>& i2, mat4 mmodel)
{
    SMesh a = create_mesh_v(v1, i1, mat4(1.f));
    SMesh b = create_mesh_v(v2, i2, mmodel);

    return meshes_intersect(a, b);

}

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>

namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> SMesh;

// A 3D grid cell key for spatial hashing
struct GridCell {
    int x, y, z;

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Hash function for GridCell
struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1) ^ (std::hash<int>()(cell.z) << 2);
    }
};

// A single triangle
struct Triangle2 {
    int index;        // Index in the triangle list
    glm::vec3 v[3];   // Triangle vertices
};

// Axis-Aligned Bounding Box
struct AABB {
    glm::vec3 min, max;

    AABB() {
        float inf = std::numeric_limits<float>::infinity();
        min = glm::vec3(inf, inf, inf);
        max = glm::vec3(-inf, -inf, -inf);
    }

    void expand(const glm::vec3& point) {
        min = glm::min(min, point);
        max = glm::max(max, point);
    }

    bool overlaps(const AABB& other) const {
        return (min.x <= other.max.x && max.x >= other.min.x) &&
            (min.y <= other.max.y && max.y >= other.min.y) &&
            (min.z <= other.max.z && max.z >= other.min.z);
    }

    glm::vec3 center() const {
        return (min + max) * 0.5f;
    }
};

// Uniform Grid for spatial hashing
struct UniformGrid {
    float cellSize; // Size of each cell
    std::unordered_map<GridCell, std::vector<int>, GridCellHash> grid;

    GridCell pointToCell(const glm::vec3& point) const {
        return GridCell{
            static_cast<int>(std::floor(point.x / cellSize)),
            static_cast<int>(std::floor(point.y / cellSize)),
            static_cast<int>(std::floor(point.z / cellSize))
        };
    }

    void insertTriangle(const Triangle2& tri) {
        AABB bounds;
        for (const auto& vertex : tri.v) {
            bounds.expand(vertex);
        }

        GridCell minCell = pointToCell(bounds.min);
        GridCell maxCell = pointToCell(bounds.max);

        for (int x = minCell.x; x <= maxCell.x; ++x) {
            for (int y = minCell.y; y <= maxCell.y; ++y) {
                for (int z = minCell.z; z <= maxCell.z; ++z) {
                    grid[GridCell{ x, y, z }].push_back(tri.index);
                }
            }
        }
    }
};

// Triangle-triangle intersection test (simplified, replace with actual intersection test)
static bool trianglesIntersect(const Triangle2& t1, const Triangle2& t2) {
    AABB b1, b2;
    for (const auto& v : t1.v) b1.expand(v);
    for (const auto& v : t2.v) b2.expand(v);
    return b1.overlaps(b2);
}

// Collision detection between two grids
static bool checkCollision(const UniformGrid& grid1, const UniformGrid& grid2,
    const std::vector<Triangle2>& triangles1,
    const std::vector<Triangle2>& triangles2) {
    for (const auto& [cell, triIndices1] : grid1.grid) {
        auto it = grid2.grid.find(cell);
        if (it == grid2.grid.end()) continue;

        const auto& triIndices2 = it->second;
        for (int t1 : triIndices1) {
            for (int t2 : triIndices2) {
                if (trianglesIntersect(triangles1[t1], triangles2[t2])) {
                    return true; // Collision found
                }
            }
        }
    }
    return false;
}

#endif