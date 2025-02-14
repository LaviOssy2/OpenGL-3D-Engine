#ifndef SceneObject_H
#define SceneObject_H

#include "../Utils/BaseUtils.h"

#include <GLFW/glfw3.h>
#include "../OpenGL_Rendering/RenderShader.h"

class ObjectManager;

using namespace std;
using namespace glm;

class SceneObject
{
public:
	ObjectManager* manager;

	string name;
	void SetName(string const& name) { this->name = name; }

	vec3 position;
	vec3 rotation;
	vec3 scale;

	float pitch;
	float yaw;
	float roll;

	SceneObject(string const& name, ObjectManager* manager, vec3 pos = vec3(0.f), vec3 rot = vec3(0.f), vec3 scl = vec3(1.f))
	{
		this->name = name;
		this->manager = manager;
		this->position = pos;
		this->rotation = rot;
		this->scale = scl;
	};

	virtual void SetPosition(vec3 newPos) {};
	virtual void SetRotation(vec3 newRot) {};
	virtual void SetScale(vec3 newScale) {};
	virtual void Create(GLFWwindow* window) {};
	virtual void Update(GLFWwindow* window) {};
	virtual void FixedUpdate(GLFWwindow* window, float fixedDeltaTime) {};
	virtual void Draw() {};
	virtual Shader* GetShader() { return NULL; };
};

#endif


