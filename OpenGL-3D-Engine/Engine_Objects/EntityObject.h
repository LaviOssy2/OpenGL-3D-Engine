#ifndef EntityObject_H
#define EntityObject_H

#include "ModelObject.h"
class EntityObject : public ModelObject
{
public:
	EntityObject(string const& name, Model* model, ObjectManager* manager, int k = 1) : ModelObject(name, model, manager, k) {};

	vec3 velocity = vec3(0.f);
	vec3 dir = vec3(0.0f);
	bool onGround = false;
	bool moveUp = false;
	bool firstTouch = false;
	float xsp = 0.f;
	float ysp = 0.f;
	float zsp = 0.f;
	int yTo = 0;
	int yawTo = 0.f;
	int pitchTo = 0.f;
	int rollTo = 0.f;


	void Create(GLFWwindow* window);
	void Update(GLFWwindow* window);
	void FixedUpdate(GLFWwindow* window, float fixedDeltaTime);
	//void Draw();
};

#endif // !1
