// native-lib.cpp
#include <jni.h>
#include <GLES3/gl3.h>
#include <cmath>

// Vertex shader: applies a simple translate of -5 on Z and a projection matrix
static const char* VERT_SHADER = R"(
    #version 300 es
    layout(location=0) in vec3 aPos;
    uniform mat4 uMVP;
    void main() {
        vec4 pos = vec4(aPos, 1.0);
        pos.z -= 5.0;
        gl_Position = uMVP * pos;
    }
)";

// Fragment shader: solid color
static const char* FRAG_SHADER = R"(
    #version 300 es
    precision mediump float;
    out vec4 oColor;
    void main() {
        oColor = vec4(0.4, 0.7, 1.0, 1.0);
    }
)";

// Cube vertices (8 corners)
static const float CUBE_VERTS[] = {
    -1.0f, -1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,
     1.0f, -1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f
};

// Index data: 12 triangles (two per face)
static const uint16_t CUBE_INDICES[] = {
    0,1,2,  2,3,0,  // back
    4,5,6,  6,7,4,  // front
    0,4,7,  7,3,0,  // left
    1,5,6,  6,2,1,  // right
    3,2,6,  6,7,3,  // top
    0,1,5,  5,4,0   // bottom
};

// GL objects
static GLuint program = 0;
static GLuint vao = 0;
static GLint  mvpLoc = -1;

// Surface dimensions
static int surfaceW = 0;
static int surfaceH = 1;  // avoid divide-by-zero

// Compile a shader from source
static GLuint compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    return s;
}

// Build a simple projection matrix (column-major)
static void makeProjection(float* out) {
    float aspect = float(surfaceW) / float(surfaceH);
    float fov = 45.0f * 3.14159f / 180.0f;
    float f = 1.0f / tanf(fov * 0.5f);
    float near = 0.1f;
    float far  = 100.0f;
    // zero
    for (int i = 0; i < 16; ++i) out[i] = 0.0f;
    out[0]  = f / aspect;
    out[5]  = f;
    out[10] = -(far + near) / (far - near);
    out[11] = -1.0f;
    out[14] = -(2.0f * far * near) / (far - near);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_initEngine(JNIEnv*, jobject) {
    // 1) Compile & link shaders
    GLuint vs = compileShader(GL_VERTEX_SHADER, VERT_SHADER);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, FRAG_SHADER);
    program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    
    // 2) Setup VAO/VBO/EBO
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint vbo, ebo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE_VERTS), CUBE_VERTS, GL_STATIC_DRAW);

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(CUBE_INDICES), CUBE_INDICES, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // 3) Enable depth test
    glEnable(GL_DEPTH_TEST);

    // 4) Get uniform location
    mvpLoc = glGetUniformLocation(program, "uMVP");
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_resizeEngine(JNIEnv*, jobject, jint width, jint height) {
    surfaceW = width;
    surfaceH = (height > 0 ? height : 1);
    glViewport(0, 0, surfaceW, surfaceH);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_openglandroid_MainActivity_renderEngine(JNIEnv*, jobject) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(program);

    // Build projection
    float proj[16];
    makeProjection(proj);
    glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, proj);

    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, nullptr);
    glBindVertexArray(0);
}
