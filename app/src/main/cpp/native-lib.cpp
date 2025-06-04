#include <jni.h>
#include <android/asset_manager_jni.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

#include "shader.h"
#include "player.h"
#include "player2.h"

#include "sunlight.h"
#include "touch_input.h"
#include "img_loader.h"
#include "left_background_stick.h"
#include "camera.h"

static std::unique_ptr<Shader> shader;               // your 3D model shader
static std::unique_ptr<Shader> joystick_shader;      // new joystick shader

static GLuint joystickTexID = 0;
static GLuint joystickVAO, joystickVBO;
static const float JOY_SIZE = 200.0f;      // width & height of your joystick image
static glm::mat4 orthoProj;

static std::unique_ptr<Player> player;
static std::unique_ptr<Player2> player2;

static std::unique_ptr<SunLight> sunLight;
static std::unique_ptr<LeftBGStick> joystick;

static std::unique_ptr<Camera> camera;

static int surfaceW = 0, surfaceH = 1;
static float angle = 0.0f;

#define LOG_TAG "TouchGL"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

// globals:
static JavaVM*  gJavaVM   = nullptr;
static jobject  gActivity = nullptr;

// 1) Grab the VM when the lib loads:
jint JNI_OnLoad(JavaVM* vm, void* /*reserved*/) {
    gJavaVM = vm;
    return JNI_VERSION_1_6;
}

// helper to call MainActivity.showDebug(...)
static void showFingerPosCpp(jint idx, jfloat x, jfloat y) {
    // 1) Get a JNIEnv on this thread
    JNIEnv* env = nullptr;
    gJavaVM->AttachCurrentThread(&env, nullptr);

    // 2) Find your Activity’s class
    jclass cls = env->GetObjectClass(gActivity);

    // 3) Look up the showFingerPos(int, float, float) method
    //    I   → int
    //    F   → float
    //    F   → float
    //    V   → void
    jmethodID mid = env->GetMethodID(cls, "showFingerPos", "(IFF)V");

    // 4) Call it with your C++ values
    env->CallVoidMethod(
        gActivity,
        mid,
        idx,
        x,
        y
    );

    // 5) Clean up the local ref to the class
    env->DeleteLocalRef(cls);

    // (Optionally, if you won’t make more JNI calls soon)
    // gJavaVM->DetachCurrentThread();
}


// … your other JNI methods …

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_TouchGLSurfaceView_nativeOnTouch(
    JNIEnv* env, jobject /*self*/,
    jint action, jint pointerIndex,
    jint pointerId, jfloat x, jfloat y, jint fingerCount)
{
    // now you can build any debug message you want:
    //showFingerPosCpp(pointerId, x, y);
    Input::UpdateTouch(pointerIndex, pointerId, x, y, fingerCount, action);

    // … plus whatever other native logic you do …
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_initEngine(
    JNIEnv* env, jobject activity, jobject assetManager)
{
    gActivity = env->NewGlobalRef(activity);

    AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);
    shader = std::make_unique<Shader>("shader/ModelShader.vs", "shader/ModelShader.fs", mgr);

    player = std::make_unique<Player>(mgr);
    sunLight = std::make_unique<SunLight>();
    player2 = std::make_unique<Player2>(mgr);
    player2->start();
    // -----------------
    // Load joystick shader

    // 2) compile the 2D textured-quad shader
    joystick_shader = std::make_unique<Shader>(
        "shader/joystick_shader.vs",
        "shader/joystick_shader.fs",
        mgr
    );
    joystick = std::make_unique<LeftBGStick>(mgr);

    camera = std::make_unique<Camera>();
    // Give it a projection matching the surface aspect ratio:
    float aspect = surfaceW > 0 ? (surfaceW / (float)surfaceH) : 1.0f;
    camera->setPerspective(45.0f, aspect, 0.1f, 100.0f);

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_resizeEngine(
    JNIEnv* /*env*/, jobject /*this*/, jint w, jint h)
{
    surfaceW = w;
    surfaceH = h > 0 ? h : 1;
    glViewport(0, 0, surfaceW, surfaceH);

    if (camera) {
        float aspect = surfaceW / (float)surfaceH;
        camera->setPerspective(45.0f, aspect, 0.1f, 100.0f);
    }

    // new: update ortho for joystick quad
    orthoProj = glm::ortho(
      0.0f, float(surfaceW),
      float(surfaceH), 0.0f,
     -1.0f, 1.0f
    );
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_renderEngine(
    JNIEnv* /*env*/, jobject /*this*/)
{
    // 1) Clear at the top of the frame
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 2) Let the camera update itself (e.g. apply rotation/translation)
    if (camera) {
        camera->update();
        camera->draw(*shader);
    }

    sunLight->getComponent<LightComponent>()->light->apply(*shader, 0);
    
    player->update();
    player->draw(*shader);
    // Before drawing:
    joystick->update(surfaceW);
    player2->update();
    player2->draw(*shader);

    // After 3D draw, replace the manual block with:
    joystick->draw(*joystick_shader, orthoProj);
}
