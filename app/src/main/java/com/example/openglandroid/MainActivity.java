package com.example.openglandroid;

import android.app.Activity;
import android.content.res.AssetManager;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.widget.TextView;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class MainActivity extends Activity implements GLSurfaceView.Renderer {
    static { System.loadLibrary("native-lib"); }

    private TouchGLSurfaceView glView;
    private TextView           debugText;
    private TextView           finger0Text;
    private TextView           finger1Text;
    // JNI hooks into your C++
    public native void initEngine(AssetManager mgr);
    public native void resizeEngine(int w, int h);
    public native void renderEngine();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // 1️⃣ Inflate the XML layout (must match the filename in res/layout/)
        setContentView(R.layout.activity_main);

        // 2️⃣ Lookup the GLSurfaceView and the TextView
        glView    = findViewById(R.id.glview);
        debugText = findViewById(R.id.debugText);
        finger0Text = findViewById(R.id.finger0Text);
        finger1Text = findViewById(R.id.finger1Text);
        // 3️⃣ Initialize your GL view
        glView.setEGLContextClientVersion(3);
        glView.setRenderer(this);

        showDebug("Count OK");
        showFingerPos(0, 0, 0);
        showFingerPos(1, 0, 0);
    }

    @Override
    public void onSurfaceCreated(GL10 unused, EGLConfig config) {
        // pass the AssetManager into native init
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

    /** Called from C++ (via JNI) to update the debug overlay. */
    public void showDebug(final String msg) {
        runOnUiThread(() -> debugText.setText(msg));
    }
    /** call this to update finger <i>idx</i>’s X,Y label */
    public void showFingerPos(final int idx, final float x, final float y) {
        runOnUiThread(() -> {
            String t = String.format("F%d: %.1f, %.1f", idx, x, y);
            if (idx == 0) {
                finger0Text.setText(t);
            } else if (idx == 1) {
                finger1Text.setText(t);
            }
        });
    }

}
