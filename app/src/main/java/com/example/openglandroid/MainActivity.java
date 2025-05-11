package com.example.openglandroid;


import android.app.Activity;
import android.opengl.GLSurfaceView;
import android.os.Bundle;

import javax.microedition.khronos.opengles.GL10;
import javax.microedition.khronos.egl.EGLConfig;

public class MainActivity extends Activity implements GLSurfaceView.Renderer {
    static { System.loadLibrary("native-lib"); }

    private GLSurfaceView glView;

    // 1) Declare your three native entry points:
    public native void initEngine();
    public native void resizeEngine(int width, int height);
    public native void renderEngine();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        glView = new GLSurfaceView(this);
        glView.setEGLContextClientVersion(3);
        glView.setRenderer(this);
        setContentView(glView);
    }

    // 2) Initialize GL (compile/link, VBO/VAO, depth test, get uniform locs)
    @Override
    public void onSurfaceCreated(GL10 unused, EGLConfig config) {
        initEngine();
        // optional: set a default clear color
        // GLES30.glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    }

    // 3) Notify your native code of size changes
    @Override
    public void onSurfaceChanged(GL10 unused, int width, int height) {
        resizeEngine(width, height);
    }

    // 4) Render a frame (your cube)
    @Override
    public void onDrawFrame(GL10 unused) {
        renderEngine();
    }
}