#include "Cube.h"
#include "../Utils/Physics.h"
#include "../Core_Entities/ObjectManager.h"

void Cube::Create(GLFWwindow* window)
{
    for (int j = 0; j < this->meshes[0].subBoxes.size(); j++)
    {
        AssignNewConvexHull(this->meshes[0].subBoxes[j]);
        //vector<vec3> newverts;
        //vector<unsigned int> newinds;
        //SimplifyConvexMeshWithIndices(this->meshes[0].subBoxes[j].currentPoints, this->meshes[0].subBoxes[j].inds, 50, newverts, newinds);

        //this->meshes[0].subBoxes[j].currentPoints = newverts;
        //this->meshes[0].subBoxes[j].inds = newinds;

        AssignOBB(this->meshes[0].subBoxes[j]);
        //cout << this->meshes[0].subBoxes[j].currentPoints.size() << " these are..\n";
    }
    this->SetEVERYTHING(vec3(0.f, 5.f, 0.f), 0, 180.f, 0, vec3(0.5f, .5f, .5f));
    //this->YAW(45.f);
}

void Cube::Update(GLFWwindow* window)
{
    //Camera* c = static_cast<Camera*>(this->manager->sceneObjects["Camera"]);
    yTo = int(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) - int(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
    yawTo = int(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) - int(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
    pitchTo = int(glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) - int(glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS);
    rollTo = int(glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) - int(glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
    /*if (pitchTo != 0 || yawTo != 0)
    {
        cout << "PITCH-------------------: " << this->pitch << "\n";
        cout << frontRotAxis.x << ", " << frontRotAxis.y << ", " << frontRotAxis.z << "\n";
        cout << rightRotAxis.x << ", " << rightRotAxis.y << ", " << rightRotAxis.z << "\n";
        cout << upRotAxis.x << ", " << upRotAxis.y << ", " << upRotAxis.z << "\n";
    }*/

    //int yTo = int(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) - int(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);

    /*if ()
    {
        velocity.y += 1.f
        moveF = true;
    }*/
    /*if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        velocity.z = 3.f;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        velocity.x = -3.f;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        velocity.x = 3.f;
    }*/
    /*if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        jump = true;
        onGround = false;
    }*/
}

void Cube::FixedUpdate(GLFWwindow* window, float fixedDeltaTime) {
    onGround = false;
    velocity.y += (float)yTo * fixedDeltaTime * 30.f;
    vec3 force = (frontRotAxis * -(float)pitchTo) + (rightRotAxis * (float)rollTo);
    force.y = 0; // Ignore vertical component for horizontal movement
    velocity += force * fixedDeltaTime * 30.f;

    // Limit velocity to avoid excessively fast movement
    float maxSpeed = 20.0f;
    if (length(velocity) > maxSpeed) {
        velocity = normalize(velocity) * maxSpeed;
    }

    ModelObject* a = static_cast<ModelObject*>(this->manager->sceneObjects["City"]);
    // Camera* c = static_cast<Camera*>(this->manager->sceneObjects["Camera"]);

    // Apply damping to horizontal velocity (x, z) to simulate friction
    velocity.x *= onGround ? 0.9f : 0.9f;  // More friction when on the ground
    velocity.z *= onGround ? 0.9f : 0.9f;
    velocity.y *= onGround ? 0.9f : 0.9f;


    // Update position based on velocity

    // Smooth pitch and roll adjustments
    float pitchAdjustment = length(velocity) * 60.f * fixedDeltaTime;
    float rollAdjustment = length(velocity) * 60.f * fixedDeltaTime;

    //velocity.y += -9.81f * fixedDeltaTime; // Gravity force
    // Collision Detection
    if (length(velocity) > 0.001f)
        if (CollidesWith(window, a, velocity * fixedDeltaTime)) {
            onGround = true;
            if (length(velocity) > 0)
                while (!CollidesWith(window, a, normalize(velocity) * fixedDeltaTime)) {

                    dir = normalize(velocity) * fixedDeltaTime;
                    this->SetPosition(dir);
                }


            velocity = vec3(0.f);
        }

    /*
    // Apply gravity
    if (!onGround) {

    }
    else
    {
        if (length(velocity) > 0)

        rollTo = 0;
        pitchTo = 0;
        yawTo = 0;

        //// a trial to
        //vec3 d = vec3(velocity.x, 0, velocity.z);
        //if (length(d) > 0.1f)
        //{
        //}


    }
    */
    this->SetPosition(velocity * fixedDeltaTime);



    float pF = 0;
    if (pitchTo == 0) {
        pF = -sign(this->pitch) * std::min(abs(this->pitch), pitchAdjustment);
    }
    else if (this->pitch > -20.f && this->pitch < 20.f) {
        pF = pitchTo * 40.f * fixedDeltaTime;
    }
    mat4 mod = mat4(1.f);

    if (abs(pF) > 0.001)
    {
        this->pitch += pF;
        mod = BuildNewModelMatrix();
        if (!CollidesWithV(window, a, mod))
            UpdateVerticesFromModelMatrix(mod);
        else
            this->pitch -= pF;
        this->GetShader()->use();
        this->GetShader()->setMat4("model", mod);
    }

    float rF = 0;
    if (rollTo == 0) {
        rF = -sign(this->roll) * std::min(abs(this->roll), rollAdjustment);
    }
    else if (this->roll > -20.f && this->roll < 20.f) {
        rF = rollTo * 40.f * fixedDeltaTime;
    }

    //float rF = rollTo == 0 ? -sign(this->roll) * std::min(abs(this->roll), rollAdjustment) : std::clamp(rollTo * 40.f * fixedDeltaTime, -20.f, 20.f);
    if (abs(rF) > 0.001)
    {
        this->roll += rF;
        mod = BuildNewModelMatrix();
        if (!CollidesWithV(window, a, mod))
            UpdateVerticesFromModelMatrix(mod);
        else
            this->roll -= rF;
        this->GetShader()->use();
        this->GetShader()->setMat4("model", mod);
    }


    float yF = yawTo * 40.f * fixedDeltaTime;
    if (abs(yF) > 0.0001)
    {
        this->yaw += yF;
        mod = BuildNewModelMatrix();
        if (!CollidesWithV(window, a, mod))
            UpdateVerticesFromModelMatrix(mod);
        else
            this->yaw -= yF;
        this->GetShader()->use();
        this->GetShader()->setMat4("model", mod);
    }



    //
    //else
      //  this->yaw -= yF;

    //if (!CollidesWithV(window, a, BuildNewModelMatrix()))
    //{
    //    if (pitchTo == 0) {
    //        this->PITCH(-sign(this->pitch) * std::min(abs(this->pitch), pitchAdjustment));
    //    }
    //    else if (this->pitch > -20.f && this->pitch < 20.f) {
    //        this->PITCH(pitchTo * 40.f * fixedDeltaTime);
    //    }

    //    if (rollTo == 0) {
    //        this->ROLL(-sign(this->roll) * std::min(abs(this->roll), rollAdjustment));
    //    }
    //    else if (this->roll > -20.f && this->roll < 20.f) {
    //        this->ROLL(rollTo * 40.f * fixedDeltaTime);
    //    }

    //    // Adjust yaw for smooth rotation
    //    this->YAW(yawTo * 40.f * fixedDeltaTime);
    //}
}


//void Cube::Draw() 
//{
//
//}