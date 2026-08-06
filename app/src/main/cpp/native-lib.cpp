#include <jni.h>
#include <android/asset_manager_jni.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>
// native-lib.cpp (at very top, before any includes or code)
#include "globals.h"
#include "object_manager.h"
#include "shader.h"
#include "player.h"
#include "player2.h"
#include "sunlight.h"
#include "left_background_stick.h"
#include "left_front_stick.h"
#include "camera.h"
#include "cube_object.h"

// std::unique_ptr<ObjectManager> scene;

// std::unique_ptr<Shader> shader;
// std::unique_ptr<Shader> joystick_shader;

// GLuint joystickTexID = 0;
// GLuint joystickVAO;
// GLuint joystickVBO;
// const float JOY_SIZE = 200.0f;
// glm::mat4 orthoProj;

// std::unique_ptr<Player>  player;
// std::unique_ptr<Player2> player2;

// std::unique_ptr<SunLight>    sunLight;
// std::unique_ptr<LeftBGStick> joystick;

// std::unique_ptr<Camera> camera;

// int   surfaceW = 0;
// int   surfaceH = 1;
// float angle    = 0.0f;


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

std::unique_ptr<ObjectManager> createOriginalScene(AAssetManager* mgr) {
    auto sceneObj = std::make_unique<ObjectManager>();
    
    auto leftBG = std::make_unique<LeftBGStick>(mgr);
    LeftBGStick* bgPtr = leftBG.get();
    auto leftFront = std::make_unique<LeftFrontStick>(mgr, bgPtr);
    LeftFrontStick* frontPtr = leftFront.get();

    sceneObj->addObject("Player", std::make_unique<Player>(mgr, frontPtr));
    sceneObj->addObject("Sun-Light", std::make_unique<SunLight>());
    sceneObj->addObject("Player2", std::make_unique<Player2>(mgr));
    sceneObj->addObject("LeftStick", std::move(leftBG));
    sceneObj->addObject("LeftFrontStick", std::move(leftFront));
    sceneObj->addObject("Camera", std::make_unique<Camera>(true));
    return sceneObj;
}



std::unique_ptr<ObjectManager> createCubeScene(AAssetManager* mgr) {
    auto sceneObj = std::make_unique<ObjectManager>();
    sceneObj->addObject("Cube", std::make_unique<CubeObject>(mgr));
    sceneObj->addObject("Sun-Light", std::make_unique<SunLight>());
    sceneObj->addObject("Camera", std::make_unique<Camera>(false));
    return sceneObj;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_initEngine(
    JNIEnv* env, jobject activity, jobject assetManager)
{
    gActivity = env->NewGlobalRef(activity);

    AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);
    shader = std::make_unique<Shader>("shader/ModelShader.vs", "shader/ModelShader.fs", mgr);
    
    // Load joystick shader
    joystick_shader = std::make_unique<Shader>(
        "shader/joystick_shader.vs",
        "shader/joystick_shader.fs",
        mgr
    );

    // Set primary scene to the 1x1x1 Cube scene
    scene = createOriginalScene(mgr);
    scene->startAll();

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

    // if (camera) {
        //     camera->setPerspective(45.0f, aspect, 0.1f, 100.0f);
        // }
    scene.get()->resize();    

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

    // // 2) Let the camera update itself (e.g. apply rotation/translation)
    // if (camera) {
    //     camera->update();
    //     camera->draw();
    // }

    scene.get()->updateAll();
    scene.get()->drawAll();
    // scene.get()->getObject("Camera")->update();
    // scene.get()->getObject("Camera")->draw();
    // scene.get()->getObject("Sun-Light")->update();
    // scene.get()->getObject("Player")->update();
    // scene.get()->getObject("Player")->draw();
    //player->update();
    //player->draw();
    // Before drawing:
    // joystick->update();
    // player2->update();
    // player2->draw();

    // // // After 3D draw, replace the manual block with:
    // joystick->draw();
}

