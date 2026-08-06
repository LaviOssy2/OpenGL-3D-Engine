#include "globals.h"
#include "object_manager.h"
#include "shader.h"
#include "player.h"
#include "player2.h"
#include "sunlight.h"
#include "left_background_stick.h"
#include "camera.h"

std::unique_ptr<ObjectManager> scene;
std::unique_ptr<Shader>        shader;
std::unique_ptr<Shader>        joystick_shader;

GLuint      joystickTexID = 0;
GLuint      joystickVAO;
GLuint      joystickVBO;
const float JOY_SIZE      = 200.0f;
glm::mat4   orthoProj;

std::unique_ptr<Player>  player;
std::unique_ptr<Player2> player2;

std::unique_ptr<SunLight>    sunLight;
std::unique_ptr<LeftBGStick> joystick;

std::unique_ptr<Camera> camera;

int   surfaceW = 0;
int   surfaceH = 1;
float angle    = 0.0f;
