package com.example.openglandroid;

import android.content.Context;
import android.util.AttributeSet;
import android.opengl.GLSurfaceView;
import android.view.MotionEvent;

public class TouchGLSurfaceView extends GLSurfaceView {
    static { System.loadLibrary("native-lib"); }

    // ① Called when you do `new TouchGLSurfaceView(context)`
    public TouchGLSurfaceView(Context ctx) {
        super(ctx);
        init();
    }

    // ② Called when inflating from XML (this is what fixes your crash)
    public TouchGLSurfaceView(Context ctx, AttributeSet attrs) {
        super(ctx, attrs);
        init();
    }

    private void init() {
        //setEGLContextClientVersion(3);
        // assume your Activity implements GLSurfaceView.Renderer
        //setRenderer((Renderer)getContext());
        setEGLContextClientVersion(3);
    }

    private native void nativeOnTouch(int action,
                                      int pointerIndex,
                                      int pointerId,
                                      float x,
                                      float y,
                                      int fingerCount);

    @Override
    public boolean onTouchEvent(MotionEvent ev) {
        int action = ev.getActionMasked();
        int count  = ev.getPointerCount();

        for (int i = 0; i < count; i++) {
            final int idx = i;
            int id = ev.getPointerId(i);
            float x = ev.getX(i);
            float y = ev.getY(i);
            int fingerCount = count;

            // For each finger, report its position
            queueEvent(() -> nativeOnTouch(action, idx, id, x, y, fingerCount));
        }

        return true;
    }


}
