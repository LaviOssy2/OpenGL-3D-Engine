#ifndef CAMERA_H
#define CAMERA_H

#include "../Engine_Objects/ModelObject.h"
#include "../Engine_Objects/SceneObject.h"
#include "../Engine_Objects/CameraObject.h"


enum Camera_Movement {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	UP,
	RollRight
};

const float SPEED = 5.f;
const float SENSITIVITY = 0.1f;

class Camera : public CameraObject
{
public:
	Camera(ObjectManager* manager);

	// returns the view matrix calculated using Euler Angles and the LookAt Matrix
	mat4 GetViewMatrix();

	virtual void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true);
	virtual void Update(GLFWwindow* window);
	virtual void FixedUpdate(GLFWwindow* window, float fixedDeltaTime);

	//bool DetectCollision(ModelObject* model);

private:
	void updateCameraVectors();

	float speed;
	float jump = 20.f;
	float mouseSensitivity;
	float zoom;

	vec3 velocity = vec3(0.f);
	vec3 dir = vec3(0.0f);

	const float friction = 0.9f; // Friction factor between 0 (no movement) and 1 (no friction)
	glm::vec3 gravity = WorldUp * -9.81f;// glm::vec3(0.0f, -9.81f, 0.0f); // Adjust for gravity effect
	bool onGround = false;
	bool canMove = true;
};

#endif