#pragma once
#include <memory>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>

// forward declarations – no heavy headers here
class ObjectManager;
class Shader;
class Player;
class Player2;
class SunLight;
class LeftBGStick;
class Camera;

// global single-instances
extern std::unique_ptr<ObjectManager> scene;
extern std::unique_ptr<Shader>        shader;
extern std::unique_ptr<Shader>        joystick_shader;

extern GLuint      joystickTexID;
extern GLuint      joystickVAO;
extern GLuint      joystickVBO;
extern const float JOY_SIZE;
extern glm::mat4   orthoProj;

extern std::unique_ptr<Player>        player;
extern std::unique_ptr<Player2>       player2;
extern std::unique_ptr<SunLight>      sunLight;
extern std::unique_ptr<LeftBGStick>   joystick;
extern std::unique_ptr<Camera>        camera;

extern int   surfaceW;
extern int   surfaceH;
extern float angle;
