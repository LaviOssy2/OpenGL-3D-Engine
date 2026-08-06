#ifndef ObjectManager_H
#define ObjectManager_H

#include "../Engine_Objects/SceneObject.h"
#include "../Engine_Objects/ModelObject.h"
#include "../Engine_Objects/EntityObject.h"
#include "../Engine_Objects/CameraObject.h"

#include "Camera.h"



class ObjectManager
{
public:
	unsigned int numOfObjects = 0;
	// settings
	const unsigned int SCR_WIDTH = 1200;
	const unsigned int SCR_HEIGHT = 1000;

	float lastX = SCR_WIDTH / 2.0f;
	float lastY = SCR_HEIGHT / 2.0f;
	bool firstMouse = true;

	Camera* camera; // Fast access to camera

	// Map of objects in the scene
	map<string, SceneObject*> sceneObjects;

	ObjectManager();

	template<typename T, typename std::enable_if<std::is_base_of<ModelObject, T>::value>::type* = nullptr>
	T* Instantiate(GLFWwindow* window, string const& name, string const& path) {
		// Creates the model for the model-object
		Model* objectModel = new Model(path);
		// Creates the actual object using (name as a key), (model), (owner-helpful)
		T* object = new T(name, objectModel, this, name == "City" ? 100 : 1);

		// Adds the object to the scene list
		this->sceneObjects[name] = object;
		numOfObjects++;

		this->sceneObjects[name]->Create(window);

		return object;
	}

	template<typename T, typename std::enable_if<std::is_base_of<ModelObject, T>::value>::type* = nullptr>
	T* Instantiate(GLFWwindow* window, string const& name, vector<vec3> verts, vector<unsigned int> inds) {
		// Creates the model for the model-object
		Model* objectModel = new Model(verts, inds);
		// Creates the actual object using (name as a key), (model), (owner-helpful)
		T* object = new T(name, objectModel, this, 1);

		// Adds the object to the scene list
		this->sceneObjects[name] = object;
		numOfObjects++;

		this->sceneObjects[name]->Create(window);

		return object;
	}

	void AssignCamera(Camera* cam);

	void Create(GLFWwindow* window);
	void Update(GLFWwindow* window);
	void FixedUpdate(GLFWwindow* window, float fixedDeltaTime);
	void Draw(GLFWwindow* window);

	void mouse_callback(GLFWwindow* window, double xposIn, double yposIn);
};

#endif
