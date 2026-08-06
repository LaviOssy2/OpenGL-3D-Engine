#include "Camera.h"
#include "../Utils/Physics.h"
#include "ObjectManager.h"

Camera::Camera(ObjectManager* manager) :
    CameraObject("Camera", manager), speed(SPEED), mouseSensitivity(SENSITIVITY), zoom(ZOOM)
{
    updateCameraVectors();

    // Setting default location
    this->position.y = 15.f;
    this->position.z = 5.f;
}

// returns the view matrix calculated using Euler Angles and the LookAt Matrix
mat4 Camera::GetViewMatrix()
{
    return lookAt(this->position, this->position + this->front, this->up);
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch)
{
    xoffset *= this->mouseSensitivity;
    yoffset *= this->mouseSensitivity;

    yaw += xoffset;
    pitch += yoffset;

    if (constrainPitch) {
        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;
    }

    updateCameraVectors();
}

void Camera::updateCameraVectors()
{
    glm::vec3 Front;
    Front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    Front.y = sin(glm::radians(pitch));
    Front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(Front);
    right = glm::normalize(glm::cross(front, WorldUp));  // Use new WorldUp

}

void Camera::Update(GLFWwindow* window) {
    dir = vec3(0.f);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        dir += front;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        dir += -front;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        dir += -right;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        dir += right;
    /*if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        up.x += 0.001f;
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        up.x -= 0.001f;
    */

    //this->SetPosition(this->position + dir);
}

void Camera::FixedUpdate(GLFWwindow* window, float fixedDeltaTime)
{
    ////dir.y = 0;
    //if (canMove)
        //velocity += dir * speed * fixedDeltaTime;
    //// Apply gravity only when not on the ground
    //if (!onGround) {
    //    //velocity += gravity * fixedDeltaTime;
        //velocity.x *= friction;
       // velocity.z *= friction;
    //}
    //else {
    //    /*cout << glfwGetTime() - st << "\n";
    //    cout << sqrt(2 * abs(sp.y - position.y) / 9.81) << "\n";
    //    cout << "\n";*/
    //    velocity *= friction; // Apply friction when on ground
    //    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
    //        velocity.y = jump;
    //    }
    //}
    EntityObject* a = static_cast<EntityObject*>(this->manager->sceneObjects["Drone"]);
    //  this->position += this->dir * this->speed * fixedDeltaTime;
    this->position = a->position + vec3(0.f, 7.f, 10.f);
    //    this->front = a->frontRotAxis;
    //    this->right = a->rightRotAxis;
    //    this->up = vec3(a->roll,0 glm::normalize(glm::cross(right, front));
    //    cout << this->up.x << ", " << this->up.y << ", " << this->up.z << "\n";
        // vec3(a->position.x, a->position.y + 3.f, a->position.z + 3.f);//dir* speed* fixedDeltaTime;
    //int max = this->manager->sceneObjects.size();
    //for (auto object : this->manager->sceneObjects) {
    //    if (ModelObject* o = dynamic_cast<ModelObject*>(object.second))
    //    {
    //        this->DetectCollision(o);
    //    }
    //}
}

//bool Camera::DetectCollision(ModelObject* model)
//{
//    vector<unsigned int> relevantMeshes = model->GetClosestMeshes(this->name);// coll.getClosestMeshesInRange(camera.Position, 100.f);
//    bool b1 = false;
//    bool b2 = true;
//    for (int i = 0; i < relevantMeshes.size(); i++) {
//        if (rayMeshCollision(this->position, glm::vec3(0.f, -1.f, 0.f), model->GetVertices(relevantMeshes[i]), model->GetIndices(relevantMeshes[i]), 1.5f)) {
//            onGround = true;
//            velocity.y = 0.0f;  // Reset vertical velocity on collision
//            b1 = true;
//
//            model->model->meshes[relevantMeshes[i]].color = vec4(1.f, 0.f, 0.f, 1.f);
//        }
//
//        if (rayMeshCollision(this->position, glm::normalize(dir), model->GetVertices(relevantMeshes[i]), model->GetIndices(relevantMeshes[i]), 1.f)) {
//            velocity.x = 0.f;
//            velocity.z = 0.f;
//            b2 = false;
//            model->model->meshes[relevantMeshes[i]].color = vec4(1.f, 0.f, 0.f, 1.f);
//        }
//    }
//    onGround = b1;
//    canMove = b2;
//    return false;
//}
