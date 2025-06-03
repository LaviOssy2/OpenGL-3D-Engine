#include <jni.h>
#include <android/asset_manager_jni.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

#include "shader.h"
#include "player.h"
#include "sunlight.h"
#include "touch_input.h"
#include "texture_utils.h"

static std::unique_ptr<Shader> shader;               // your 3D model shader
static std::unique_ptr<Shader> joystick_shader;      // new joystick shader

static GLuint joystickTexID = 0;
static GLuint joystickVAO, joystickVBO;
static const float JOY_SIZE = 200.0f;      // width & height of your joystick image
static glm::mat4 orthoProj;

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

    // -----------------
    // Load joystick shader
    joystickTexID = LoadTextureFromAsset(mgr, "images/joystick.png");

    // 2) compile the 2D textured-quad shader
    joystick_shader = std::make_unique<Shader>(
        "shader/joystick_shader.vs",
        "shader/joystick_shader.fs",
        mgr
    );

    // 3) build a screen-space quad (two triangles)
    float quadVertices[] = {
      // pos.x, pos.y,   u,   v
      100, 100,       0, 0,
      300, 100,       1, 0,
      300, 300,       1, 1,

      100, 100,       0, 0,
      300, 300,       1, 1,
      100, 300,       0, 1
    };
    glGenVertexArrays(1, &joystickVAO);
    glGenBuffers(1, &joystickVBO);
    glBindVertexArray(joystickVAO);
    glBindBuffer(GL_ARRAY_BUFFER, joystickVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    // pos attrib
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    // uv attrib
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          (void*)(2 * sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    // ——————————————————————————

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

    // 2) Draw your 3D world
    glm::mat4 V = glm::lookAt(glm::vec3(0,0,6), glm::vec3(0,0,0), glm::vec3(0,1,0));
    glm::mat4 P = glm::perspective(glm::radians(45.0f),
                                   surfaceW/(float)surfaceH,
                                   0.1f, 100.0f);
    shader->use();
    shader->setMat4("view",       V);
    shader->setMat4("projection", P);

    sunLight->getComponent<LightComponent>()->light->apply(*shader, 0);
    
    player->update();
    player->draw(*shader);

    // 3) Now draw your joystick on top
    auto touch0 = Input::GetTouch(0);
    if ( touch0 && joystickTexID && joystick_shader ) {
      // turn off depth-­test so it always draws over the 3D
      glDisable(GL_DEPTH_TEST);
      // (optionally also glDepthMask(GL_FALSE);)

      // rebuild & upload VBO as before…
      float cx = touch0->position.x,
            cy = touch0->position.y,
            half = JOY_SIZE*0.5f;
      float x1 = cx-half, y1 = cy-half,
            x2 = cx+half, y2 = cy+half;
      float quadVertices[] = {
        x1,y1, 0,0,
        x2,y1, 1,0,
        x2,y2, 1,1,
        x1,y1, 0,0,
        x2,y2, 1,1,
        x1,y2, 0,1
      };
      glBindBuffer(GL_ARRAY_BUFFER, joystickVBO);
      glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(quadVertices), quadVertices);
      glBindBuffer(GL_ARRAY_BUFFER, 0);

      // draw the textured quad
      joystick_shader->use();
      joystick_shader->setMat4("uProjection", orthoProj);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, joystickTexID);
      joystick_shader->setInt("uTexture", 0);

      glBindVertexArray(joystickVAO);
      glDrawArrays(GL_TRIANGLES, 0, 6);
      glBindVertexArray(0);

      // re-enable depth test for next frame
      glEnable(GL_DEPTH_TEST);
      // (if you disabled depth writes: glDepthMask(GL_TRUE);)
    }
}
