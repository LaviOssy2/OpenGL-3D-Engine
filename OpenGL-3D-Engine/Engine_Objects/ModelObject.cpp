#include "ModelObject.h"
#include "../Core_Entities/ObjectManager.h"

ModelObject::ModelObject(string const& name, Model* model, ObjectManager* manager, int k, bool convex, bool precise) :
    SceneObject(name, manager), model(model)
{
    cout << this->model->meshes.size() << "\n";

    //int i = 3;
    for (int i = 0; i < this->model->meshes.size(); i++)
    {
        this->meshes[i].verts = this->model->meshes[i].orgPositions;
        this->meshes[i].inds = this->model->meshes[i].orgIndices;
        if (k > 1)
            GenerateKMeansClusters(this->meshes[i], k);
        else
        {
            // In case k == 1 no clustering should be applied
            this->meshes[i].subBoxes.clear();
            this->meshes[i].subBoxes.push_back(CollisionBox());
            for (auto& box : this->meshes[i].subBoxes) {
                box.orgPoints = this->meshes[i].verts;
                box.currentPoints = this->meshes[i].verts;
                box.inds = this->meshes[i].inds;
            }
        }
        for (int j = 0; j < this->meshes[i].subBoxes.size(); j++)
        {
            if (convex)
                AssignNewConvexHull(this->meshes[i].subBoxes[j]);

            if (!precise)
                AssignOBB(this->meshes[i].subBoxes[j]);
        }
        //cout << "mesh " << i << "has " << this->meshes[i].subBoxes.size() << "size boxes. \n";
    }

    this->UpdateModelMatrix();

    //cout << "Max verts: " << v << "\n";
    //cout << "Max inds: " << n << "\n";

    cout << "---------------------------------------------------\n";

}


void ModelObject::GenerateKMeansClusters(MeshCurrentData& meshData, int k)
{
    auto assignments = AssignKMeansClusters(meshData.verts, meshData.inds, k);
    meshData.subBoxes = AssignCollisionBoxes(assignments, meshData.verts, k);
}

void ModelObject::SetEVERYTHING(vec3 newPos, float p, float y, float r, vec3 scale)
{
    this->position += newPos;
    this->pitch += p;
    this->yaw += y;
    this->roll += r;
    this->scale.x *= scale.x;
    this->scale.y *= scale.y;
    this->scale.z *= scale.z;

    matModel = UpdateModelMatrix();
}
void ModelObject::SetPosition(vec3 newPos)
{
    this->position += newPos;
    if (length(newPos) > 0.001f)
        UpdateModelMatrix();      // Recalculate model matrix with new position
}
void ModelObject::PITCH(float val)
{
    this->pitch += val;

    if (abs(val) > 0.001f)
        UpdateModelMatrix();      // Recalculate model matrix with new rotation
}
void ModelObject::YAW(float val)
{
    this->yaw += val;

    if (abs(val) > 0.001f)
        UpdateModelMatrix();
}
void ModelObject::ROLL(float val)
{
    this->roll += val;

    if (abs(val) > 0.001f)
        UpdateModelMatrix();      // Recalculate model matrix with new rotation
}
void ModelObject::SetScale(vec3 newScale)
{
    this->scale.x *= newScale.x;
    this->scale.y *= newScale.y;
    this->scale.z *= newScale.z;
    UpdateModelMatrix();      // Recalculate model matrix with new scale
}
void ModelObject::UpdateAxises()
{
    glm::vec3 Front;
    Front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch)) * -1;
    Front.y = sin(glm::radians(pitch));
    Front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));

    frontRotAxis = glm::normalize(Front);
    rightRotAxis = glm::normalize(glm::cross(frontRotAxis, vec3(0.f, 1.f, 0.f)));  // Use new WorldUp
    upRotAxis = glm::normalize(glm::cross(rightRotAxis, frontRotAxis));
}

mat4 ModelObject::UpdateModelMatrix()
{
    this->shader->use();

    //// Reset model to identity matrix and apply transformations in the correct order
    matModel = mat4(1.0f);

    //// Apply position, then rotation, then scale
    matModel = translate(matModel, position);

    glm::vec3 Front;
    Front.x = -cos(glm::radians(yaw + 90)) * cos(glm::radians(pitch));
    Front.y = sin(glm::radians(pitch));
    Front.z = sin(glm::radians(yaw + 90)) * cos(glm::radians(pitch));
    frontRotAxis = glm::normalize(Front);
    rightRotAxis = glm::normalize(glm::cross(frontRotAxis, vec3(0.f, 1.f, 0.f)));  // Use new WorldUp

    matModel = rotate(matModel, radians(this->pitch), rightRotAxis); // X rotation
    matModel = rotate(matModel, radians(this->roll), frontRotAxis); // Z rotation
    matModel = rotate(matModel, radians(this->yaw), vec3(0.f, 1.f, 0.f)); // Y rotation

    matModel = glm::scale(matModel, scale);

    for (int k = 0; k < this->meshes.size(); k++)
    {
        for (int i = 0; i < this->meshes[k].subBoxes.size(); i++) {
            vec3 currentMin = vec3(FLT_MAX);
            vec3 currentMax = vec3(-FLT_MAX);
            for (int j = 0; j < this->meshes[k].subBoxes[i].currentPoints.size(); j++)
            {
                this->meshes[k].subBoxes[i].currentPoints[j] = vec3(matModel * glm::vec4(this->meshes[k].subBoxes[i].orgPoints[j], 1.0f));
                currentMin = glm::min(currentMin, this->meshes[k].subBoxes[i].currentPoints[j]);
                currentMax = glm::max(currentMax, this->meshes[k].subBoxes[i].currentPoints[j]);
            }
            this->meshes[k].subBoxes[i].min = currentMin;
            this->meshes[k].subBoxes[i].max = currentMax;
            //free(this->meshes[k].subBoxes[i].mesh);
            /*if (this->meshes[k].subBoxes[i].mesh == nullptr) {
                this->meshes[k].subBoxes[i].mesh = new SMesh();
            }
            */// this is problematic line - remove it and it works
            //this->meshes[k].subBoxes[i].mesh = new SMesh(create_mesh(this->meshes[k].subBoxes[i].currentPoints, this->meshes[k].subBoxes[i].inds, vec3(0.f)));
            /*if (this->meshes[k].subBoxes[i].mesh == NULL) {
                this->meshes[k].subBoxes[i].mesh = new SMesh(create_mesh(this->meshes[k].subBoxes[i].currentPoints, this->meshes[k].subBoxes[i].inds, vec3(0.f)));
            }
            else {
                update_mesh_vertices(*this->meshes[k].subBoxes[i].mesh, this->meshes[k].subBoxes[i].currentPoints);
            }*/

        }
    }

    // Pass the updated model matrix to the shader
    this->shader->setMat4("model", matModel);
    return matModel;
}
static bool CheckAABBCollision(glm::vec3 min1, glm::vec3 max1, glm::vec3 min2, glm::vec3 max2) {
    // Check overlap on the X axis
    if (max1.x < min2.x || max2.x < min1.x) return false;
    // Check overlap on the Y axis
    if (max1.y < min2.y || max2.y < min1.y) return false;
    // Check overlap on the Z axis
    if (max1.z < min2.z || max2.z < min1.z) return false;

    // If no separating axis, the AABBs are colliding
    return true;
}


bool ModelObject::CollidesWith(GLFWwindow* window, ModelObject* other, vec3 TO)
{
    // Track all potential candidates for detailed collision check
    std::vector<std::pair<int, int>> candidateBoxes; // Pair of mesh index and sub-box index

    // Iterate through all meshes and sub-boxes in the other object
    for (int i = 0; i < other->meshes.size(); i++)
    {
        for (int j = 0; j < other->meshes[i].subBoxes.size(); j++)
        {
            CollisionBox& box1 = other->meshes[i].subBoxes[j];

            // Compute the distance from this object's position (offset by TO) to the current sub-box
            float distanceToBox = DistanceToBoundingBox(this->position + TO, box1);

            for (int n = 0; n < this->meshes.size(); n++)
            {
                for (int m = 0; m < this->meshes[n].subBoxes.size(); m++)
                {
                    CollisionBox& box2 = this->meshes[n].subBoxes[m];
                    if (distanceToBox <= length(normalize(TO) * (vec3(box2.max - box2.min) / 2.f)))
                    {
                        //SMesh& a = *box1.mesh;
                        //SMesh& b = *box2.mesh;
                        //

                        //add_to_mesh(b, TO);
                        //b.collect_garbage(); // Clean up unused vertices or faces
                        ////PMP::build_face_adjacency(b); // Rebuild adjacency and connectivity data

                        //if (!CGAL::is_valid_polygon_mesh(a)) {
                        //    std::cerr << "Mesh1 is invalid!" << std::endl;
                        //    return false;
                        //}

                        //if (!CGAL::is_valid_polygon_mesh(b)) {
                        //    std::cerr << "Mesh2 is invalid!" << std::endl;
                        //    return false;
                        //}

                        // Perform precise triangle-level collision detection
                        vector<unsigned int>& i1 = box1.inds;
                        vector<unsigned int>& i2 = box2.inds;
                        vector<vec3>& v1 = box1.currentPoints;
                        vector<vec3> v2;
                        for (vec3& v : box2.currentPoints)
                            v2.push_back(v + TO);
                        std::vector<Triangle2> triangles1, triangles2;
                        for (size_t i = 0; i < i1.size(); i += 3) {
                            triangles1.push_back({ static_cast<int>(i / 3), {v1[i1[i]], v1[i1[i + 1]], v1[i1[i + 2]]} });
                        }
                        for (size_t i = 0; i < i2.size(); i += 3) {
                            triangles2.push_back({ static_cast<int>(i / 3), {v2[i2[i]], v2[i2[i + 1]], v2[i2[i + 2]]} });
                        }

                        UniformGrid grid1{ 1.0f }, grid2{ 1.0f };
                        for (const auto& tri : triangles1) grid1.insertTriangle(tri);
                        for (const auto& tri : triangles2) grid2.insertTriangle(tri);
                        bool uniformCollision = checkCollision(grid1, grid2, triangles1, triangles2);

                        if (uniformCollision) {// meshes_intersect(a, b)) {
                            return true;
                        }
                    }
                }

            }
        }
    }
    /*

    // No candidate boxes, no collision
    if (candidateBoxes.empty())
    {
        return false;
    }

    // Perform detailed collision detection with all candidates
    for (const auto& candidate : candidateBoxes)
    {
        int meshIndex = candidate.first;
        int subBoxIndex = candidate.second;

        CollisionBox& closestBox = other->meshes[meshIndex].subBoxes[subBoxIndex];

        for (int n = 0; n < this->meshes.size(); n++)
        {
            for (int m = 0; m < this->meshes[n].subBoxes.size(); m++)
            {
                CollisionBox& box2 = this->meshes[n].subBoxes[m];

                // Perform precise triangle-level collision detection
                if (DetectCollision(closestBox.currentPoints, closestBox.inds, box2.currentPoints, box2.inds, TO))
                {
                    return true;
                }
            }
        }
    }
    */

    // No collision found after detailed checks
    return false;
}
mat4 ModelObject::BuildNewModelMatrix()
{
    mat4 matModel = mat4(1.0f);

    //// Apply position, then rotation, then scale
    matModel = translate(matModel, position);

    glm::vec3 Front;
    Front.x = -cos(glm::radians(yaw + 90)) * cos(glm::radians(pitch));
    Front.y = sin(glm::radians(pitch));
    Front.z = sin(glm::radians(yaw + 90)) * cos(glm::radians(pitch));
    frontRotAxis = glm::normalize(Front);
    rightRotAxis = glm::normalize(glm::cross(frontRotAxis, vec3(0.f, 1.f, 0.f)));  // Use new WorldUp

    matModel = rotate(matModel, radians(this->pitch), rightRotAxis); // X rotation
    matModel = rotate(matModel, radians(this->roll), frontRotAxis); // Z rotation
    matModel = rotate(matModel, radians(this->yaw), vec3(0.f, 1.f, 0.f)); // Y rotation

    matModel = glm::scale(matModel, scale);

    return matModel;
};
bool ModelObject::CollidesWithV(GLFWwindow* window, ModelObject* other, mat4 mmodel)
{
    // Track all potential candidates for detailed collision check
    std::vector<std::pair<int, int>> candidateBoxes; // Pair of mesh index and sub-box index

    // Iterate through all meshes and sub-boxes in the other object
    for (int i = 0; i < other->meshes.size(); i++)
    {
        for (int j = 0; j < other->meshes[i].subBoxes.size(); j++)
        {
            CollisionBox& box1 = other->meshes[i].subBoxes[j];

            // Compute the distance from this object's position (offset by TO) to the current sub-box

            float distanceToBox = DistanceToBoundingBox(vec3(mmodel * vec4(0.f, 0.f, 0.f, 1.f)), box1);

            for (int n = 0; n < this->meshes.size(); n++)
            {
                for (int m = 0; m < this->meshes[n].subBoxes.size(); m++)
                {
                    CollisionBox& box2 = this->meshes[n].subBoxes[m];
                    // Include boxes within a reasonable threshold for detailed checking
                    if (distanceToBox <= length(box2.max - box2.min) / 2.f)
                    {
                        // Perform precise triangle-level collision detection
                        if (DetectCollisionV(box1.currentPoints, box1.inds, box2.orgPoints, box2.inds, mmodel))
                        {
                            return true;
                        }
                    }
                }

                //candidateBoxes.push_back({ i, j });
            }
        }
    }
    return false;
}