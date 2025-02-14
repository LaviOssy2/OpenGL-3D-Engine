#ifndef ModelObject_H
#define ModelObject_H

#include "../Utils/BaseUtils.h"
#include "../Utils/Physics.h"
#include "../OpenGL_Rendering/RenderModel.h"

#include "SceneObject.h"

struct MeshCurrentData {
    std::vector<vec3> verts;
    std::vector<unsigned int> inds;
    std::vector<CollisionBox> subBoxes;
};

class ModelObject : public SceneObject
{
public:
    // Physical model
    Model* model;
    // Meshes deterimined by the physical model (from external app like blender)
    map<unsigned int, MeshCurrentData> meshes;
    // An efficient way instead of giving too much clusters for a mesh (for more simple meshes)
    map<int, int> suboxK = {};

    // Optimize matrix recalculation by checking for mesh movement
    bool moves = false;

    // Seems redundent but I will keep it for now
    // -----------------------
    vec3 frontRotAxis;
    vec3 rightRotAxis;
    vec3 upRotAxis;
    // -----------------------


    ModelObject(string const& name, Model* model, ObjectManager* manager, int k = 1, bool convex = false, bool precise = false);

    virtual void Update(GLFWwindow* window) {}; // Static Model
    virtual void FixedUpdate(GLFWwindow* window) {};
    virtual void Draw() { this->model->Draw(*this->shader); }

    void GenerateKMeansClusters(MeshCurrentData& meshData, int k);

    // Actions
    void SetEVERYTHING(vec3 newPos, float p, float y, float r, vec3 scale);
    void SetPosition(vec3 newPos);
    void PITCH(float val);
    void YAW(float val);
    void ROLL(float val);
    void SetScale(vec3 newScale);
    void UpdateAxises();
    bool CollidesWith(GLFWwindow* window, ModelObject* other, vec3 TO = vec3(0.f));


    void SetK(int mesh, int k)
    {
        this->suboxK[mesh] = k;
    }
    bool IsCuboid(const MeshCurrentData& meshData) {
        map<int, set<int>> neighbors;
        set<float> uniqueEdgeLengths;

        for (int i = 0; i < meshData.inds.size(); i += 3) {
            int v0 = meshData.inds[i];
            int v1 = meshData.inds[i + 1];
            int v2 = meshData.inds[i + 2];

            neighbors[v0].insert(v1);
            neighbors[v0].insert(v2);

            neighbors[v1].insert(v0);
            neighbors[v1].insert(v2);

            neighbors[v2].insert(v0);
            neighbors[v2].insert(v1);
        }

        for (const auto& [vertex, vertexNeighbors] : neighbors) {
            for (int neighbor : vertexNeighbors) {
                vec3 diff = meshData.verts[vertex] - meshData.verts[neighbor];
                float length = glm::length(diff); // Assuming `diff.length()` returns the Euclidean distance
                //uniqueEdgeLengths.insert();
                bool found = false;
                for (float existingLength : uniqueEdgeLengths) {
                    if (std::abs(existingLength - length) < 0.001f) {
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    uniqueEdgeLengths.insert(length);
                }
            }
        }
        // A cuboid has 3 unique edge lengths (width, height, depth)
        return uniqueEdgeLengths.size() <= 6;
    }


    Shader* GetShader()
    {
        return shader;
    }
    vector<vec3> GetVertices(unsigned int mesh)
    {
        return this->meshes[mesh].verts;
    };
    vector<unsigned int> GetIndices(unsigned int mesh)
    {
        return this->meshes[mesh].inds;
    };
    mat4 BuildNewModelMatrix();
    void UpdateVerticesFromModelMatrix(mat4 mmodel)
    {
        for (int k = 0; k < this->meshes.size(); k++)
        {
            for (int i = 0; i < this->meshes[k].subBoxes.size(); i++) {
                vec3 currentMin = vec3(FLT_MAX);
                vec3 currentMax = vec3(-FLT_MAX);
                for (int j = 0; j < this->meshes[k].subBoxes[i].currentPoints.size(); j++)
                {
                    this->meshes[k].subBoxes[i].currentPoints[j] = vec3(mmodel * glm::vec4(this->meshes[k].subBoxes[i].orgPoints[j], 1.0f));
                    currentMin = glm::min(currentMin, this->meshes[k].subBoxes[i].currentPoints[j]);
                    currentMax = glm::max(currentMax, this->meshes[k].subBoxes[i].currentPoints[j]);
                }
                this->meshes[k].subBoxes[i].min = currentMin;
                this->meshes[k].subBoxes[i].max = currentMax;
            }
        }
        matModel = mmodel;
    };


    // <newVerts, <mines, maxes>>
    pair<vector<vector<vector<vec3>>>, pair<vector<vec3>, vector<vec3>>> GetUpdatedVertices()
    {
        vector<vector<vector<vec3>>> newVerts(this->meshes.size());
        vector<vec3> mines;
        vector<vec3> maxes;
        for (int k = 0; k < this->meshes.size(); k++)
        {
            for (int i = 0; i < this->meshes[k].subBoxes.size(); i++) {
                vec3 currentMin = vec3(FLT_MAX);
                vec3 currentMax = vec3(-FLT_MAX);
                newVerts[i].resize(this->meshes[k].subBoxes.size());
                for (int j = 0; j < this->meshes[k].subBoxes[i].currentPoints.size(); j++)
                {
                    newVerts[k][i].push_back(vec3(matModel * glm::vec4(this->meshes[k].subBoxes[i].orgPoints[j], 1.0f)));
                    currentMin = glm::min(currentMin, this->meshes[k].subBoxes[i].currentPoints[j]);
                    currentMax = glm::max(currentMax, this->meshes[k].subBoxes[i].currentPoints[j]);
                }
                mines.push_back(currentMin);
                mines.push_back(currentMax);
            }
        }
        return make_pair(newVerts, make_pair(mines, maxes));
    };
    bool CollidesWithV(GLFWwindow* window, ModelObject* other, mat4 mmodel);

private:
    mat4 UpdateModelMatrix();
    Shader* shader = new Shader("OpenGL_Rendering/ModelShader.vs", "OpenGL_Rendering/ModelShader.fs");;
    glm::mat4 matModel = glm::mat4(1.0f);
};

#endif

