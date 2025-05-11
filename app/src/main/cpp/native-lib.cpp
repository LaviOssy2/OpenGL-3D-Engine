#include <jni.h>
#include <GLES3/gl3.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <memory>
#include <vector>

#include "Model.h"
#include "Shader.h"

using namespace std;

static unique_ptr<Model> model;
static unique_ptr<Shader> shader;
static float angle = 0.0f;
static GLint mvpLoc;
static int surfaceW = 0, surfaceH = 1;

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_initEngine(
    JNIEnv* env,
    jobject /* this */,
    jobject assetManager) {

    AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);
    shader = make_unique<Shader>(
        "shaders/ModelShader.vs",
        "shaders/ModelShader.fs",
        mgr);

    // Cube data
    vector<glm::vec3> verts = {
        {-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1},
        {-1,-1, 1}, {1,-1, 1}, {1,1, 1}, {-1,1, 1}
    };
    vector<unsigned int> inds = {
        0,1,2, 2,3,0,
        4,5,6, 6,7,4,
        0,4,7, 7,3,0,
        1,5,6, 6,2,1,
        3,2,6, 6,7,3,
        0,1,5, 5,4,0
    };
    model = make_unique<Model>(verts, inds);

    glEnable(GL_DEPTH_TEST);
    mvpLoc = glGetUniformLocation(shader->ID, "uMVP");
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_resizeEngine(
    JNIEnv*,
    jobject,
    jint width,
    jint height) {
    surfaceW = width;
    surfaceH = height > 0 ? height : 1;
    glViewport(0, 0, surfaceW, surfaceH);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_renderEngine(
    JNIEnv*,
    jobject) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shader->use();

    angle += 0.01f;
    glm::mat4 modelMat = glm::rotate(
        glm::mat4(1.0f),
        angle,
        glm::vec3(0,1,0));
    glm::mat4 view = glm::lookAt(
        glm::vec3(0,0,6),
        glm::vec3(0,0,0),
        glm::vec3(0,1,0));
    glm::mat4 proj = glm::perspective(
        glm::radians(45.0f),
        surfaceW / (float)surfaceH,
        0.1f,
        100.0f);
    glm::mat4 mvp = proj * view * modelMat;
    glUniformMatrix4fv(
        mvpLoc,
        1,
        GL_FALSE,
        &mvp[0][0]);

    model->Draw(*shader);
}