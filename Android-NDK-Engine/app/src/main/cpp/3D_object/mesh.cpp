#include "mesh.h"

Mesh::Mesh(const std::vector<Vertex>& verts, const std::vector<unsigned int>& inds)
    : vertices(verts), indices(inds)
{
    setupMesh();
}

void Mesh::Draw(Shader& shader) {
    // Activate the shader program so the GPU uses this shader pipeline.
    // This shader must be compatible with the vertex attributes configured in the VAO.
    shader.use();

    // Set the 'model' matrix uniform in the shader to the identity matrix,
    // meaning no transformation will be applied to this mesh.
    // (In a real application, you would often pass a dynamic transform here.)
// shader.setMat4("model",      glm::mat4(1.0f));              // identity for debug
// shader.setMat4("view",       glm::mat4(1.0f));              // identity
// shader.setMat4("projection", glm::mat4(1.0f));              // identity means clip space == NDC
// shader.setVec3("lightPos",   glm::vec3(0.0f, 1.0f, 0.0f));  // your debug light

    // Bind the VAO (Vertex Array Object), which:
    // - Automatically rebinds the VBO (vertex data) and EBO (index data)
    // - Restores the layout configuration (e.g., position, normal, texCoord)
    // - Tells OpenGL exactly how to fetch and interpret the vertex data
    glBindVertexArray(VAO);

    // Issue a draw call using the element buffer (EBO).
    // - GL_TRIANGLES: interpret every 3 indices as a triangle
    // - indices.size(): total number of indices to draw
    // - GL_UNSIGNED_INT: type of the indices
    // - 0: offset into the EBO (start from the beginning)
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);

    // Unbind the VAO to avoid accidentally modifying it later.
    glBindVertexArray(0);
}


void Mesh::setupMesh() {
    // Requests (from OpenGL) a unique ID for a Vertex Array Object (VAO), 
    // which will store all the configuration/state needed to render this mesh 
    // (like buffer bindings and vertex attribute layouts).
    glGenVertexArrays(1, &VAO); 

    // Requests unique IDs for the Vertex Buffer Object (VBO) and Element Buffer Object (EBO).
    glGenBuffers(1, &VBO); // The VBO will store the actual vertex data (positions, normals, texcoords),
    glGenBuffers(1, &EBO); // while the EBO will store index data that defines how vertices connect to form triangles.

    // Bind the Vertex Array Object (VAO) so that all following configuration
    // (buffers and vertex attribute setup) will be stored in it.
    glBindVertexArray(VAO); // it is like a container for the VBO and EBO - close to gen but here we bind it

    // Bind the VBO and EBO to the GL_ARRAY_BUFFER and GL_ELEMENT_ARRAY_BUFFER targets respectively.
    // This means that all following buffer operations will affect the VBO and EBO.
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW); // static draw means that the data will not change often
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Position
    glEnableVertexAttribArray(0);
    // Define how OpenGL should interpret the vertex data for attribute location 0:
    // 0 → Attribute index in the shader layout (e.g., layout(location = 0) in the vertex shader).
    // 3 → This attribute is a vec3 (x, y, z), so it has 3 components.
    // GL_FLOAT → Each component is a float.
    // GL_FALSE → Don’t normalize the values (use them as raw floats).
    // sizeof(Vertex) → The total size (stride) of one Vertex in memory — tells OpenGL how to move from one vertex to the next.
    // (void*)offsetof(Vertex, position) → Offset in bytes from the start of the Vertex struct to the `position` field.
    // → This tells OpenGL where to find position data within each vertex.
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));

    // Normal - same as above
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    // TexCoord - same as above
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoord));

    // Unbind the VBO and EBO to prevent accidental modification.
    glBindVertexArray(0);
}
