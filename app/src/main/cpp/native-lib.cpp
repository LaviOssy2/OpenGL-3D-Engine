#include <jni.h>
#include <android/asset_manager_jni.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

#include "renderer/shader.h"
#include "renderer/model.h"

using namespace std;

static unique_ptr<Shader> shader;
static unique_ptr<Model>  model;
static int   surfaceW = 0, surfaceH = 1;
static float angle    = 0.0f;

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_initEngine(
    JNIEnv* env, jobject /* this */, jobject assetManager)
{
    AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);
    shader = make_unique<Shader>(
        "shader/ModelShader.vs",
        "shader/ModelShader.fs",
        mgr
    );
    model = make_unique<Model>("models/cube.obj", mgr);

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_resizeEngine(
    JNIEnv* /*env*/, jobject /*this*/,
    jint w, jint h)
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
    glm::mat4 M = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0,1,0));
    glm::mat4 V = glm::lookAt(
        glm::vec3(0,0,6),
        glm::vec3(0,0,0),
        glm::vec3(0,1,0)
    );
    glm::mat4 P = glm::perspective(
        glm::radians(45.0f),
        surfaceW / float(surfaceH),
        0.1f, 100.0f
    );

    shader->use();
    shader->setMat4("model",      M);
    shader->setMat4("view",       V);
    shader->setMat4("projection", P);

    model->Draw(*shader, surfaceW, surfaceH);
}
