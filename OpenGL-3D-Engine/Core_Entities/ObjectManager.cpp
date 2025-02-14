#include "ObjectManager.h"

ObjectManager::ObjectManager()
{
}

//ModelObject* ObjectManager::InstantiateModel(string const& name, string const& path, vec3 pos, vec3 rot, vec3 scl)
//{
//	Model* objectModel = new Model(path);
//	ModelObject* EntityObject = new ModelObject(name, numOfObjects, objectModel, this);
//	this->sceneObjects[name] = EntityObject;
//	//this->sceneObjects[name]->SetID(numOfObjects);
//	//this->sceneObjects[name]->SetName(name);
//	numOfObjects++;
//
//	EntityObject->SetPosition(pos);
//	EntityObject->SetRotation(rot);
//	EntityObject->SetScale(scl);
//	
//
//	return EntityObject;
//}

//template<typename T, typename std::enable_if<std::is_base_of<ModelObject, T>::value>::type*>
//T* ObjectManager::Instantiate(string const& path, vec3 pos, vec3 rot, vec3 scl)
//{
//	
//}
void ObjectManager::AssignCamera(Camera* cam) {
	// Creates the camera
	this->camera = cam;
	// Adds to the list of objects
	this->sceneObjects["Camera"] = this->camera;

	numOfObjects++;
}
void ObjectManager::Create(GLFWwindow* window)
{
	for (auto object : sceneObjects)
		object.second->Create(window);
}
void ObjectManager::Update(GLFWwindow* window)
{
	for (auto object : sceneObjects)
		object.second->Update(window);
}
void ObjectManager::FixedUpdate(GLFWwindow* window, float fixedDeltaTime)
{
	for (auto object : sceneObjects)
		object.second->FixedUpdate(window, fixedDeltaTime);
}
void ObjectManager::Draw(GLFWwindow* window)
{
	mat4 projection = perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.001f, 1000.0f);
	mat4 view = camera->GetViewMatrix();

	for (auto object : sceneObjects) {
		Shader* s = object.second->GetShader();
		if (s != NULL) {
			s->use();
			s->setMat4("projection", projection);
			s->setMat4("view", view);
			object.second->Draw();
		}
	}
}
void ObjectManager::mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
	float xpos = static_cast<float>(xposIn);
	float ypos = static_cast<float>(yposIn);

	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

	lastX = xpos;
	lastY = ypos;

	this->camera->ProcessMouseMovement(xoffset, yoffset);
}