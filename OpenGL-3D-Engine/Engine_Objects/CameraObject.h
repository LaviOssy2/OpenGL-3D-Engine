#ifndef CameraObject_H
#define CameraObject_H

#include "SceneObject.h"
#include "ModelObject.h"

// Default camera values
const float ROLL = 0.0f;
const float PITCH = 0.0f;
const float YAW = -90.0f;
const float ZOOM = 45.0f;

const vec3 WorldUp = vec3(0.f, 1.f, 0.f); // If we would like to change the axis - change the updateCameraVectors also

class CameraObject : public SceneObject
{
public:

	// In Degrees
	float roll;
	float pitch;
	float yaw;

	float zoom;


	vec3 front;
	vec3 up;
	vec3 right;

	CameraObject(string const& name, ObjectManager* manager) :
		SceneObject(name, manager), roll(ROLL), pitch(PITCH), yaw(YAW), zoom(ZOOM), up(WorldUp)
	{
		updateCameraVectors();

		// Setting default location
		this->position.y = 15.f;
		this->position.z = 5.f;
	}

	// returns the view matrix calculated using Euler Angles and the LookAt Matrix
	mat4 GetViewMatrix()
	{
		return lookAt(this->position, this->position + this->front, this->up);
	}

	virtual void Update(GLFWwindow* window) {};
	virtual void FixedUpdate(GLFWwindow* window, float fixedDeltaTime) {};

	//bool DetectCollision(ModelObject* model);

private:
	void updateCameraVectors()
	{
		glm::vec3 Front;
		Front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
		Front.y = sin(glm::radians(pitch));
		Front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
		front = glm::normalize(Front);
		right = glm::normalize(glm::cross(front, WorldUp));  // Use new WorldUp
	}
};

#endif