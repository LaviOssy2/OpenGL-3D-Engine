#include <jni.h>
#include <android/asset_manager_jni.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

#include "shader.h"
#include "player.h"
#include "sunlight.h"

static std::unique_ptr<Shader> shader;
static std::unique_ptr<Player> player;
static std::unique_ptr<SunLight> sunLight;
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
    showFingerPosCpp(pointerId, x, y);

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

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_resizeEngine(
    JNIEnv* /*env*/, jobject /*this*/, jint w, jint h)
{
    surfaceW = w;
    surfaceH = h > 0 ? h : 1;
    glViewport(0, 0, surfaceW, surfaceH);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_renderEngine(
    JNIEnv* /*env*/, jobject /*this*/)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    angle += 0.01f;
    glm::mat4 M = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0, 1, 0));
    glm::mat4 V = glm::lookAt(glm::vec3(0, 0, 6), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
    glm::mat4 P = glm::perspective(glm::radians(45.0f), surfaceW / float(surfaceH), 0.1f, 100.0f);

    shader->use();
    shader->setMat4("model", M);          // World transform (if needed)
    shader->setMat4("view", V);
    shader->setMat4("projection", P);

    // Apply light properties
    sunLight->getComponent<LightComponent>()->light->apply(*shader, 0);

    // Draw the player
    player->getComponent<ModelComponent>()->model->Draw(*shader, surfaceW, surfaceH);
}
