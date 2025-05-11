package com.example.openglandroid;

import android.app.Activity;
import android.content.res.AssetManager;
import android.opengl.GLSurfaceView;
import android.os.Bundle;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class MainActivity extends Activity implements GLSurfaceView.Renderer {
    static { System.loadLibrary("native-lib"); }

    private GLSurfaceView glView;

    // Pass AssetManager into native
    public native void initEngine(AssetManager assetManager);
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

    @Override
    public void onSurfaceCreated(GL10 unused, EGLConfig config) {
        initEngine(getAssets());
    }

    @Override
    public void onSurfaceChanged(GL10 unused, int width, int height) {
        resizeEngine(width, height);
    }

    @Override
    public void onDrawFrame(GL10 unused) {
        renderEngine();
    }
}