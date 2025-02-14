#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../Core_Entities/ObjectManager.h"
#include <iostream>
#include "Cube.h"

ObjectManager manager;
glm::vec3 HSVtoRGB(float h, float s, float v) {
    float r, g, b;
    int i = int(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
    case 0: r = v, g = t, b = p; break;
    case 1: r = q, g = v, b = p; break;
    case 2: r = p, g = v, b = t; break;
    case 3: r = p, g = q, b = v; break;
    case 4: r = t, g = p, b = v; break;
    case 5: r = v, g = p, b = q; break;
    }
    return glm::vec3(r, g, b);
}
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    manager.mouse_callback(window, xposIn, yposIn);
}
int main()
{
    // ---------------------------------------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(manager.SCR_WIDTH, manager.SCR_HEIGHT, "Engine2", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    //glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    stbi_set_flip_vertically_on_load(true);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    // --------------------------------------------------------------

    manager.AssignCamera(new Camera(&manager));

    float deltaTime = 0.0f;
    float smoothedDeltaTime = 0.0f; // Initialize smoothed deltaTime
    float lastFrame = static_cast<float>(glfwGetTime());

    const float fixedTimeStep = 0.016f;  // 60 updates per second
    float accumulator = 0.0f;
    float alpha = 0.1f; // Smoothing factor for deltaTime

    Cube* c = manager.Instantiate<Cube>(window, "Drone", "Room1/Cube/Cube.obj");

    ModelObject* m = manager.Instantiate<ModelObject>(window, "City", "Room1/City/City.obj");
    //m->model->meshes[1].color = vec4(vec3(1, 0, 0), 1.f);
    m->SetScale(vec3(20.f));

    /*

    //int k = 2;
    for (int k = 0; k < m->meshes.size(); k++)
        for (int j = 0; j < m->meshes[k].subBoxes.size(); j++) {
            // Access the current sub-box
            const auto& subBox = m->meshes[k].subBoxes[j];

            // Instantiate a cube model
            ModelObject* x = manager.Instantiate<ModelObject>(window, "HE" + to_string(k) + to_string(j), "Cube/Cube.obj");

            // Calculate position as the center of the bounding box
            glm::vec3 center = (subBox.min * 1.0001f + subBox.max * 1.0001f) / 2.f;
            x->SetPosition(center);

            // Calculate scale as half the size of the bounding box dimensions
            glm::vec3 scale = (subBox.max * 1.0001f - subBox.min * 1.0001f) / 2.f;
            x->SetScale(scale);

            // Assign a random color for visualization
            x->model->meshes[0].color = glm::vec4(
                (float)j / (float)(j < m->meshes[k].subBoxes.size()-1),                      // Random red channel
                static_cast<float>(j) / m->meshes[k].subBoxes.size(), // Gradient green channel
                RandRange(0, 1),                             // Random blue channel
                0.6f                                         // Semi-transparent alpha
            );
        }

    */

    //}

    //for (int k = 0; k < m->meshes.size(); k++)


/*
       int k = 2;
        for (int j = 0; j < m->meshes[k].subBoxes.size(); j++)
        {
            std::vector<glm::vec3> verts; ;
            for (auto v : m->meshes[k].subBoxes[j].currentPoints)
               verts.push_back(v*1.1f);
            cout << verts.size() << "\n";
            vector<unsigned int> inds = m->meshes[k].subBoxes[j].inds;
            unsigned int max = *max_element(inds.begin(), inds.end());
            if (max <= 3)
            cout << inds.size() << ", maxL " << max << "\n";

            ModelObject* x = manager.Instantiate<ModelObject>(window, "HE" + to_string(k) + to_string(j), verts, inds);
            x->model->meshes[0].color = vec4(RandRange(0, 1), j/ m->meshes[k].subBoxes.size(), RandRange(0, 1), .6f);
        }

*/
//std::vector<glm::vec3> verts;// = c->meshes[0].subBoxes[0].currentPoints;
//for (auto v : c->meshes[0].subBoxes[0].currentPoints)
//    verts.push_back(c->position + (v - c->position) * 0.99f);
//vector<unsigned int> inds = c->meshes[0].subBoxes[0].inds;
//DebugCube* x = manager.Instantiate<DebugCube>(window, "HEy" + to_string(0) + to_string(0), verts, inds);
//
//Shader* s1 = x->GetShader();
//Shader* s2 = c->GetShader();
//cout << "pointer of x: " << &s1 << "\n";
//cout << "pointer of c: " << &s2 << "\n";

    glClearColor(0.05f, 0.05f, 0.5f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    manager.Draw(window);


    while (!glfwWindowShouldClose(window))
    {
        // Calculate smoothed deltaTime based on actual frame time
        float frameDeltaTime = glfwGetTime() - lastFrame;
        lastFrame = glfwGetTime();
        deltaTime = alpha * frameDeltaTime + (1.0f - alpha) * deltaTime;
        // Accumulate time to manage fixed updates
        accumulator += deltaTime;

        manager.Update(window);

        while (accumulator >= fixedTimeStep) {
            manager.FixedUpdate(window, fixedTimeStep);
            accumulator -= fixedTimeStep;
        }

        glClearColor(0.05f, 0.05f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        manager.Draw(window);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    std::cout << "Hello World!\n";
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


