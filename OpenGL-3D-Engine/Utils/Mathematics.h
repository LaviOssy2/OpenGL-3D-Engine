#ifndef Mathematics_H
#define Mathematics_H

#include "BaseUtils.h"

#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertexSet.h>

#include <random>

#define GLM_ENABLE_EXPERIMENTAL
#include <vector>
#include <glm/gtx/matrix_decompose.hpp> // For glm::outerProduct
#include <Eigen/Dense>                 // For Eigen decomposition

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>

namespace PMP = CGAL::Polygon_mesh_processing;

// Type definitions
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> SMesh;

// Helper function to create CGAL Surface_mesh from vertices and indices
static SMesh create_mesh(const std::vector<glm::vec3>& vertices, const std::vector<unsigned int>& indices, vec3 TO) {
    SMesh mesh;

    // Convert glm::vec3 to CGAL::Point_3
    std::vector<SMesh::Vertex_index> vertex_handles;
    for (const auto& v : vertices) {
        vertex_handles.push_back(mesh.add_vertex(Point(v.x + TO.x, v.y + TO.y, v.z + TO.z)));
    }

    // Add faces using triangle indices
    for (size_t i = 0; i < indices.size() - 2; i += 3) {
        mesh.add_face(vertex_handles[indices[i]],
            vertex_handles[indices[i + 1]],
            vertex_handles[indices[i + 2]]);
    }

    return mesh;
}
static void add_to_mesh(SMesh& mesh, vec3 TO) {
    for (auto vi : mesh.vertices()) {
        Point& p = mesh.point(vi); // Access the vertex position
        p = Point(p.x() + TO.x, p.y() + TO.y, p.z() + TO.z); // Add the translation vector
    }
}



static void update_mesh_vertices(SMesh& mesh, const std::vector<glm::vec3>& updatedVertices) {
    auto vi = mesh.vertices_begin();
    for (const auto& vertex : updatedVertices) {
        if (vi == mesh.vertices_end()) break;
        mesh.point(*vi) = Point(vertex.x, vertex.y, vertex.z);
        ++vi;
    }
}
static void update_mesh_faces(SMesh& mesh, const std::vector<unsigned int>& indices) {
    mesh.clear(); // Clear existing faces
    std::vector<SMesh::Vertex_index> vertexHandles(mesh.number_of_vertices());
    size_t idx = 0;
    for (auto vi : mesh.vertices()) {
        vertexHandles[idx++] = vi;
    }

    for (size_t i = 0; i < indices.size() - 2; i += 3) {
        mesh.add_face(vertexHandles[indices[i]],
            vertexHandles[indices[i + 1]],
            vertexHandles[indices[i + 2]]);
    }
}
static void update_mesh(SMesh& mesh,
    const std::vector<glm::vec3>& updatedVertices,
    const std::vector<unsigned int>& indices) {
    // Update vertices
    auto vi = mesh.vertices_begin();
    for (const auto& vertex : updatedVertices) {
        if (vi == mesh.vertices_end()) break;
        mesh.point(*vi) = Point(vertex.x, vertex.y, vertex.z);
        ++vi;
    }

    // Rebuild faces
    update_mesh_faces(mesh, indices);
}

static SMesh create_mesh_v(const std::vector<glm::vec3>& vertices, const std::vector<unsigned int>& indices, mat4 mmodel) {
    SMesh mesh;

    // Convert glm::vec3 to CGAL::Point_3
    std::vector<SMesh::Vertex_index> vertex_handles;
    for (const auto& ov : vertices) {
        vec3 v = vec3(mmodel * glm::vec4(ov, 1.0f));
        vertex_handles.push_back(mesh.add_vertex(Point(v.x, v.y, v.z)));
    }

    // Add faces using triangle indices
    for (size_t i = 0; i < indices.size(); i += 3) {
        mesh.add_face(vertex_handles[indices[i]],
            vertex_handles[indices[i + 1]],
            vertex_handles[indices[i + 2]]);
    }

    return mesh;
}

// Perform collision detection
static bool meshes_intersect(SMesh& mesh1, SMesh& mesh2) {
    // If intersection mesh is not empty, collision detected
    return PMP::do_intersect(mesh1, mesh2);// has_intersection && !faces(intersection_mesh).empty();
}

// Function to calculate the covariance matrix
static glm::mat3 ComputeCovarianceMatrix(const std::vector<glm::vec3>& points) {
    glm::vec3 centroid(0.0f, 0.0f, 0.0f);
    for (const auto& point : points) {
        centroid += point;
    }
    centroid /= static_cast<float>(points.size());

    glm::mat3 covarianceMatrix(0.0f);
    for (const auto& point : points) {
        glm::vec3 centered = point - centroid;
        covarianceMatrix += glm::outerProduct(centered, centered);
    }
    covarianceMatrix /= static_cast<float>(points.size());
    return covarianceMatrix;
}
// Function to compute the OBB
static std::vector<glm::vec3> ComputeOBBCorners(const std::vector<glm::vec3>& points) {
    // Step 1: Compute the covariance matrix
    glm::mat3 covarianceMatrix = ComputeCovarianceMatrix(points);

    // Step 2: Perform eigen decomposition
    Eigen::Matrix3f cov;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cov(i, j) = covarianceMatrix[i][j];
        }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    Eigen::Matrix3f eigenVectors = solver.eigenvectors(); // Principal axes

    // Step 3: Convert eigenvectors to glm vectors
    glm::vec3 axes[3] = {
        glm::vec3(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0)), // X'
        glm::vec3(eigenVectors(0, 1), eigenVectors(1, 1), eigenVectors(2, 1)), // Y'
        glm::vec3(eigenVectors(0, 2), eigenVectors(1, 2), eigenVectors(2, 2))  // Z'
    };

    // Step 4: Compute the centroid
    glm::vec3 centroid(0.0f, 0.0f, 0.0f);
    for (const auto& point : points) {
        centroid += point;
    }
    centroid /= static_cast<float>(points.size());

    // Step 5: Project points onto eigenvectors and find min/max extents
    glm::vec3 minExtents(FLT_MAX), maxExtents(-FLT_MAX);
    for (const auto& point : points) {
        glm::vec3 relative = point - centroid;
        for (int i = 0; i < 3; i++) {
            float projection = glm::dot(relative, axes[i]);
            minExtents[i] = glm::min(minExtents[i], projection);
            maxExtents[i] = glm::max(maxExtents[i], projection);
        }
    }

    // Step 6: Generate corners of the OBB
    std::vector<glm::vec3> corners;
    for (int i = -1; i <= 1; i += 2) { // -1, 1 for each axis
        for (int j = -1; j <= 1; j += 2) {
            for (int k = -1; k <= 1; k += 2) {
                glm::vec3 corner = centroid
                    + (i > 0 ? maxExtents.x : minExtents.x) * axes[0]
                    + (j > 0 ? maxExtents.y : minExtents.y) * axes[1]
                    + (k > 0 ? maxExtents.z : minExtents.z) * axes[2];
                corners.push_back(corner);
            }
        }
    }

    return corners;
}

static void ComputeConvexHull(orgQhull::Qhull& qhull, std::vector<double>& pointArray, std::vector<glm::vec3>& inputVerts, std::vector<glm::vec3>& outputVerts, std::unordered_map<int, int>& vertexIndexMap, int dimension, int xIdx, int yIdx, int zIdx, float constantZ = 0.0f)
{
    pointArray.clear();
    for (const auto& v : inputVerts) {
        pointArray.push_back(v[xIdx]);
        pointArray.push_back(v[yIdx]);
        if (dimension == 3) {
            pointArray.push_back(v[zIdx]);
        }
    }

    qhull.runQhull("", dimension, inputVerts.size(), pointArray.data(), "Qt");

    for (auto vertex : qhull.vertexList()) {
        glm::vec3 vertexPos(0.0f);
        vertexPos[xIdx] = vertex.point()[0];
        vertexPos[yIdx] = vertex.point()[1];
        vertexPos[zIdx] = (dimension == 3) ? vertex.point()[2] : constantZ;

        vertexIndexMap[vertex.point().id()] = outputVerts.size();
        outputVerts.push_back(vertexPos);
    }
}

static void BuildConvexHullPolyhedron(std::vector<glm::vec3>& inputVerts, std::vector<glm::vec3>& verts, std::vector<unsigned int>& inds) {
    orgQhull::Qhull qhull;

    glm::vec3 min = inputVerts[0], max = inputVerts[0];
    for (const auto& v : inputVerts) {
        min = glm::min(min, v);
        max = glm::max(max, v);
    }
    glm::vec3 extent = max - min;
    const float epsilon = 1e-5f;

    bool isFlatXY = extent.z < epsilon;
    bool isFlatXZ = extent.y < epsilon;
    bool isFlatYZ = extent.x < epsilon;

    std::vector<double> pointArray;
    std::unordered_map<int, int> vertexIndexMap;

    if (isFlatXY) {
        ComputeConvexHull(qhull, pointArray, inputVerts, verts, vertexIndexMap, 2, 0, 1, 2, inputVerts[0].z);
    }
    else if (isFlatXZ) {
        ComputeConvexHull(qhull, pointArray, inputVerts, verts, vertexIndexMap, 2, 0, 2, 1, inputVerts[0].y);
    }
    else if (isFlatYZ) {
        ComputeConvexHull(qhull, pointArray, inputVerts, verts, vertexIndexMap, 2, 1, 2, 0, inputVerts[0].x);
    }
    else {
        ComputeConvexHull(qhull, pointArray, inputVerts, verts, vertexIndexMap, 3, 0, 1, 2);
    }

    // Extract indices for faces
    for (auto facet : qhull.facetList()) {
        std::vector<unsigned int> faceIndices;
        for (auto vertex : facet.vertices()) {
            faceIndices.push_back(vertexIndexMap[vertex.point().id()]);
        }

        if (faceIndices.size() == 3) {
            inds.insert(inds.end(), faceIndices.begin(), faceIndices.end());
        }
        else if (faceIndices.size() > 3) {
            for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
                inds.push_back(faceIndices[0]);
                inds.push_back(faceIndices[i]);
                inds.push_back(faceIndices[i + 1]);
            }
        }
    }
}


static int RandRangeInt(int i, int j) {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(i, j);

    return (int)dist(mt);
}
static double RandRange(int i, int j) {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(i, j);

    return dist(mt);
}

struct Triangle
{
    int i0;
    int i1;
    int i2;
    vec3 center(const vector<vec3>& verts)
    {
        return (verts[i0] + verts[i0] + verts[i0]) / 3.f;
    }
};
struct TriangleHash {
    size_t operator()(const Triangle& t) const {
        // Combine hash values of all indices
        return std::hash<int>()(t.i0) ^ (std::hash<int>()(t.i1) << 1) ^ (std::hash<int>()(t.i2) << 2);
    }
};
struct TriangleEqual {
    bool operator()(const Triangle& t1, const Triangle& t2) const {
        return t1.i0 == t2.i0 && t1.i1 == t2.i1 && t1.i2 == t2.i2;
    }
};

static std::unordered_map<Triangle, int, TriangleHash, TriangleEqual> AssignKMeansClusters(vector<vec3>& inputVerts, vector<unsigned int>& inputInds, int k)
{
    vector<vec3> centers;
    unsigned int max = *max_element(inputInds.begin(), inputInds.end());
    set<int> randint;
    while (randint.size() < k)
        randint.insert(RandRange(0, max));
    for (int index : randint) {
        centers.push_back(inputVerts[index]);
    }
    std::unordered_map<Triangle, int, TriangleHash, TriangleEqual> assignments;

    bool hasChanged;
    int numof = 0;
    do
    {
        hasChanged = false;
        for (int i = 0; i < inputInds.size() - 2; i += 3)
        {
            int idx0 = inputInds[i];
            int idx1 = inputInds[i + 1];
            int idx2 = inputInds[i + 2];

            Triangle t = { idx0, idx1, idx2 };
            vec3 tCenter = (inputVerts[idx0] + inputVerts[idx1] + inputVerts[idx2]) / 3.f;
            int nearestCluster = 0;
            float minDist = distance(tCenter, centers[nearestCluster]);
            for (int cluster = 1; cluster < k; cluster++)
            {
                float tempDist = distance(tCenter, centers[cluster]);
                if (minDist > tempDist) {
                    nearestCluster = cluster;
                    minDist = tempDist;
                }
            }
            if (assignments[t] != nearestCluster)
            {
                assignments[t] = nearestCluster;
                hasChanged = true;
            }
        }

        // Accumulate new centers
        vector<vec3> accCenter(k, vec3(0.f));
        vector<int> count(k, 0);

        for (const auto& assignment : assignments)
        {
            accCenter[assignment.second] += (inputVerts[assignment.first.i0] + inputVerts[assignment.first.i1] + inputVerts[assignment.first.i2]) / 3.f;
            count[assignment.second]++;
        }
        for (int i = 0; i < k; i++)
        {
            if (count[i] > 0)
                centers[i] = accCenter[i] / float(count[i]);
        }
        numof++;
    } while (hasChanged);

    return assignments;
}

static void SimplifyConvexMeshWithIndices(const std::vector<glm::vec3>& verts, const std::vector<unsigned int>& indices,
    int k, std::vector<glm::vec3>& simplifiedVerts, std::vector<unsigned int>& simplifiedIndices) {
    if (verts.size() <= static_cast<size_t>(k)) {
        simplifiedVerts = verts;
        simplifiedIndices = indices;
        return;
    }

    // Step 1: Define fixed directions for full coverage
    std::vector<glm::vec3> directions = {
        {1, 0, 0}, {-1, 0, 0},  // +X, -X
        {0, 1, 0}, {0, -1, 0},  // +Y, -Y
        {0, 0, 1}, {0, 0, -1}   // +Z, -Z
    };

    // Add extra random directions if needed
    int extraDirections = k - directions.size();
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (int i = 0; i < extraDirections; ++i) {
        glm::vec3 dir;
        do {
            dir = glm::vec3(dist(rng), dist(rng), dist(rng));
        } while (glm::length(dir) < 0.001f);
        directions.push_back(glm::normalize(dir));
    }

    // Step 2: Find extreme vertices along directions
    std::unordered_map<glm::vec3, unsigned int, Vec3Hash, Vec3Equal> vertexMap;
    simplifiedVerts.clear();
    simplifiedIndices.clear();

    for (const auto& dir : directions) {
        float maxProjection = std::numeric_limits<float>::lowest();
        glm::vec3 extremeVertex;
        unsigned int extremeIndex = 0;

        for (size_t i = 0; i < verts.size(); ++i) {
            float projection = glm::dot(verts[i], dir);
            if (abs(projection) > maxProjection) {
                maxProjection = abs(projection);
                extremeVertex = verts[i];
                extremeIndex = static_cast<unsigned int>(i);
            }
        }

        // Insert vertex and its index
        if (vertexMap.find(extremeVertex) == vertexMap.end()) {
            vertexMap[extremeVertex] = simplifiedVerts.size();
            simplifiedVerts.push_back(extremeVertex);
            simplifiedIndices.push_back(extremeIndex);
        }
    }
    // Step 3: Generate a final index buffer for the simplified mesh
    std::vector<unsigned int> finalIndices;
    for (const auto& index : indices) {
        const auto& position = verts[index];
        if (vertexMap.find(position) != vertexMap.end()) {
            finalIndices.push_back(vertexMap[position]);
        }
    }
    simplifiedIndices = finalIndices;
}
#endif