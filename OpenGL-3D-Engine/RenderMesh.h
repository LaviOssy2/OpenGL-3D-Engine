#ifndef MESH_H
#define MESH_H

//#include <glad/glad.h> // holds all OpenGL type declarations

#include "BaseUtils.h"
#include "RenderShader.h"

#define MAX_BONE_INFLUENCE 4


struct Vertex {
    // position
    glm::vec3 Position;
    // normal
    glm::vec3 Normal;
    // texCoords
    glm::vec2 TexCoords;
    // tangent
    glm::vec3 Tangent;
    // bitangent
    glm::vec3 Bitangent;
    //bone indexes which will influence this vertex
    int m_BoneIDs[MAX_BONE_INFLUENCE];
    //weights from each bone
    float m_Weights[MAX_BONE_INFLUENCE];
};

struct Texture {
    unsigned int id;
    string type;
    string path;
};

class Mesh {
public:
    // mesh Data
    vector<Vertex>       vertices;
    vector<vec3>         orgPositions;
    vector<unsigned int> indices;
    vector<unsigned int> orgIndices;
    vector<Texture>      textures;
    unsigned int VAO;
    glm::vec4 color;

    // Constructor for custom vertices and indices
    Mesh(const std::vector<glm::vec3>& verts, const std::vector<unsigned int>& inds) {
        // Convert glm::vec3 to Vertex
        for (const auto& vert : verts) {
            Vertex vertex;
            vertex.Position = vert;
            vertex.Normal = glm::vec3(0.0f, 0.0f, 1.0f); // Default normal
            vertex.TexCoords = glm::vec2(0.0f, 0.0f);    // Default texture coords
            vertices.push_back(vertex);
            orgPositions.push_back(vert);
        }
        indices = inds;
        orgIndices = inds;
        color = glm::vec4(1.f);

        setupMesh();
    }

    void DrawWhatever(Shader& shader) {
        glUniform4f(glGetUniformLocation(shader.ID, "color"), this->color.x, this->color.y, this->color.z, this->color.w);
        glUniform1i(glGetUniformLocation(shader.ID, "useTexture"), 0);
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

    }


    // constructor
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, vector<Texture> textures)
    {
        this->vertices = vertices;
        this->indices = indices;
        /*
        std::unordered_map<glm::vec3, unsigned int, Vec3Hash, Vec3Equal> vertex_map;
        std::vector<glm::vec3> unique_verts;
        std::vector<unsigned int> new_inds;
        for (const auto& v : this->vertices) {
            if (vertex_map.find(v.Position) == vertex_map.end()) {
                vertex_map[v.Position] = unique_verts.size();
                unique_verts.push_back(v.Position);
            }
        }
        */


        std::unordered_map<glm::vec3, unsigned int, Vec3Hash, Vec3Equal> vertex_map;
        std::vector<vec3> unique_verts;
        std::vector<unsigned int> new_inds;

        for (const auto& v : vertices)
        {
            glm::vec3 position = v.Position;
            //cout << position.x << ", " << position.y << ", " << position.z << "\n";
            if (vertex_map.find(position) == vertex_map.end())
            {
                vertex_map[position] = unique_verts.size();
                unique_verts.push_back(position);
            }
        }
        for (const auto& index : indices) {
            glm::vec3 position = vertices[index].Position;
            new_inds.push_back(vertex_map[position]);
        }

        /*for (int i = 0; i < indices.size() - 2; i+=3)
        {
            auto i1 = indices[i];
            auto i2 = indices[i+1];
            auto i3 = indices[i+2];

            if ()
        }*/



        // Iterate through original indices to maintain the face order
        /*for (const auto& index : indices) {
            glm::vec3 position = vertices[index].Position;
            if (vertex_map.find(position) == vertex_map.end()) {
                // Map the current size of unique_verts to this position
                vertex_map[position] = unique_verts.size();
                unique_verts.push_back(position); // push the full vertex to maintain all attributes
            }
            // Use the mapped index for this vertex
            new_inds.push_back(vertex_map[position]);
        }

        for (const auto& index : this->indices) {
            new_inds.push_back(vertex_map[this->vertices[index].Position]);
        }*/
        orgPositions = std::move(unique_verts);
        orgIndices = std::move(new_inds);

        this->textures = textures;
        this->color = glm::vec4(1.f);
        // now that we have all the required data, set the vertex buffers and its attribute pointers.
        setupMesh();
    }

    // render the mesh
    void Draw(Shader& shader)
    {
        // bind appropriate textures
        unsigned int diffuseNr = 1;
        unsigned int specularNr = 1;
        unsigned int normalNr = 1;
        unsigned int heightNr = 1;
        for (unsigned int i = 0; i < textures.size(); i++)
        {
            glActiveTexture(GL_TEXTURE0 + i); // active proper texture unit before binding
            // retrieve texture number (the N in diffuse_textureN)
            string number;
            string name = textures[i].type;
            if (name == "texture_diffuse")
                number = std::to_string(diffuseNr++);
            else if (name == "texture_specular")
                number = std::to_string(specularNr++); // transfer unsigned int to string
            else if (name == "texture_normal")
                number = std::to_string(normalNr++); // transfer unsigned int to string
            else if (name == "texture_height")
                number = std::to_string(heightNr++); // transfer unsigned int to string

            // now set the sampler to the correct texture unit
            glUniform1i(glGetUniformLocation(shader.ID, (name + number).c_str()), i);
            glUniform4f(glGetUniformLocation(shader.ID, "color"), this->color.x, this->color.y, this->color.z, this->color.w);
            glUniform1i(glGetUniformLocation(shader.ID, "useTexture"), 1);
            // and finally bind the texture
            glBindTexture(GL_TEXTURE_2D, textures[i].id);
        }

        // draw mesh
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        // always good practice to set everything back to defaults once configured.
        glActiveTexture(GL_TEXTURE0);
    }

    /*std::vector<glm::vec3> getVertsPos()
    {
        std::vector<glm::vec3> verts;
        for (int i = 0; i < this->vertices.size(); i++)
        {
            verts.push_back(this->vertices[i].Position);
        }
        return verts;
    }*/


private:
    // render data 
    unsigned int VBO, EBO;

    // initializes all the buffer objects/arrays
    void setupMesh()
    {
        // create buffers/arrays
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);
        // load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        // A great thing about structs is that their memory layout is sequential for all its items.
        // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
        // again translates to 3/2 floats which translates to a byte array.
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_DYNAMIC_DRAW);

        // set the vertex attribute pointers
        // vertex Positions
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        // vertex normals
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
        // vertex texture coords
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
        // vertex tangent
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Tangent));
        // vertex bitangent
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Bitangent));
        // ids
        glEnableVertexAttribArray(5);
        glVertexAttribIPointer(5, 4, GL_INT, sizeof(Vertex), (void*)offsetof(Vertex, m_BoneIDs));

        // weights
        glEnableVertexAttribArray(6);
        glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, m_Weights));
        glBindVertexArray(0);
    }
};

#endif