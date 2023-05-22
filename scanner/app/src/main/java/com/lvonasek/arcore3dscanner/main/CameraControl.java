package com.lvonasek.arcore3dscanner.main;

import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Point;
import android.net.Uri;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageButton;

import androidx.core.content.FileProvider;

import com.lvonasek.arcore3dscanner.BuildConfig;
import com.lvonasek.arcore3dscanner.ui.AbstractActivity;
import com.lvonasek.arcore3dscanner.vr.CardboardActivity;
import com.lvonasek.arcore3dscanner.vr.DaydreamActivity;
import com.lvonasek.gles.GLESSurfaceView;
import com.lvonasek.record.Recorder;
import com.lvonasek.utils.Compatibility;
import com.lvonasek.utils.GestureDetector;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.IntBuffer;

import javax.microedition.khronos.opengles.GL10;

public class CameraControl {

    public enum ViewMode { UNKNOWN, FACE, ORBIT, TOPDOWN, FIRST, VR, FLOORPLAN };

    private Main mActivity;
    private DistanceMeasuring mDistance;
    private Editor mEditor;
    private GestureDetector mGestureDetector;

    private ImageButton mCardboardButton;
    private ImageButton mFirstViewButton;
    private ImageButton mOrbitViewButton;
    private ImageButton mTopViewButton;

    private ViewMode mView;
    private ViewMode mPrevious;
    private float mMoveX = 0;
    private float mMoveY = 0;
    private float mMoveZ = 0;
    private float mOrbit = 0;
    private float mPitch = 0;
    private float mYawM = 0;
    private float mYawR = 0;
    private float mDiff = 0;

    private File mMakeThumbnail = null;
    private boolean mShareThumbnail = false;
    private boolean mViewMode = false;

    public CameraControl(Main activity, DistanceMeasuring distanceMeasuring, Editor editor) {
        mActivity = activity;
        mDistance = distanceMeasuring;
        mEditor = editor;

        mCardboardButton = activity.findViewById(com.lvonasek.arcore3dscanner.R.id.cardboard_button);
        mFirstViewButton = activity.findViewById(com.lvonasek.arcore3dscanner.R.id.first_button);
        mOrbitViewButton = activity.findViewById(com.lvonasek.arcore3dscanner.R.id.orbit_button);
        mTopViewButton = activity.findViewById(com.lvonasek.arcore3dscanner.R.id.topview_button);

        // Touch controller
        mGestureDetector = new GestureDetector(new GestureDetector.GestureListener()
        {
            @Override
            public boolean IsAcceptingRotation() {
                return (mView == ViewMode.TOPDOWN || mView == ViewMode.FLOORPLAN);
            }

            @Override
            public void OnDrag(float dx, float dy) {
                if (Recorder.isVideoRecording()) {
                    return;
                }
                float f = getMoveFactor();
                if (IsAcceptingRotation()) {
                    //move factor
                    f *= Math.max(1.0, mMoveZ);
                    //rotated move
                    float angle = -mYawM - mYawR;
                    Move(f, dx, dy, angle, mPitch);
                } else if ((mView == ViewMode.FACE) || (mView == ViewMode.FIRST) || (mView == ViewMode.ORBIT)) {
                    f *= mView == ViewMode.ORBIT ? -4.0f : 2.0f;
                    mPitch += dy * f;
                    mYawM -= dx * f;

                    if (mPitch > 1.57f)
                        mPitch = 1.57f;
                    if (mPitch < -1.57f)
                        mPitch = -1.57f;
                    mDistance.reset();
                }
                JNI.setView(mYawM + mYawR, mPitch, mMoveX, mMoveY, mMoveZ, mOrbit, !mViewMode);
            }

            @Override
            public void OnTwoFingerMove(float dx, float dy) {
                if (Recorder.isVideoRecording()) {
                    return;
                }
                if (mViewMode) {
                    GLESSurfaceView gl = mActivity.getGLView();
                    mDiff += Math.abs(dx) / (float)gl.getWidth() + Math.abs(dy) / (float)gl.getHeight();

                    if ((mView == ViewMode.FIRST) || (mView == ViewMode.ORBIT) || (mView == ViewMode.TOPDOWN)) {
                        //move factor
                        float f = getMoveFactor();
                        if (mView == ViewMode.ORBIT) {
                            f *= Math.max(1.0, mOrbit);
                        } else {
                            f *= Math.max(1.0, mMoveZ);
                        }
                        //rotated move
                        float angle = -mYawM - mYawR;
                        Move(f, dx, dy, angle, mPitch);

                        //apply
                        JNI.setView(mYawM + mYawR, mPitch, mMoveX, mMoveY, mMoveZ, mOrbit, !mViewMode);
                    }
                }
            }

            @Override
            public void OnTwoFingerRotation(float angle) {
                if (Recorder.isVideoRecording()) {
                    return;
                }
                if (mView != ViewMode.ORBIT) {
                    mDiff += Math.abs(mYawR - Math.toRadians(-angle));
                    mYawR = (float) Math.toRadians(-angle);
                    JNI.setView(mYawM + mYawR, mPitch, mMoveX, mMoveY, mMoveZ, mOrbit, !mViewMode);
                }
            }

            @Override
            public void OnPinchToZoom(float diff) {
                if (Recorder.isVideoRecording()) {
                    return;
                }
                mDiff += Math.abs(diff);
                if (mView == ViewMode.FACE) {
                    diff *= 0.25f;
                    mOrbit -= diff;
                    if(mOrbit < 0.25f)
                        mOrbit = 0.25f;
                    if(mOrbit > 1.5f)
                        mOrbit = 1.5f;
                } else if (mView == ViewMode.ORBIT) {

                    //limit the orbit zoom
                    mOrbit -= diff;
                    if(mOrbit < 1.0f) {
                        diff = 1.0f - mOrbit;
                        mOrbit = 1.0f;
                    } else if(mOrbit > 15) {
                        diff = 15.0f - mOrbit;
                        mOrbit = 15;
                    } else {
                        diff = 0;
                    }

                    //apply over limit movement
                    double angle = -mYawM - mYawR;
                    mMoveX += diff * Math.sin(angle) * Math.cos(mPitch);
                    mMoveY -= diff * Math.cos(angle) * Math.cos(mPitch);
                    mMoveZ += diff * Math.sin(mPitch);
                } else if (mViewMode) {
                    diff *= 0.25f * Math.max(1.0, mMoveZ);
                    double angle = -mYawM - mYawR;
                    mMoveX += diff * Math.sin(angle) * Math.cos(mPitch);
                    mMoveY -= diff * Math.cos(angle) * Math.cos(mPitch);
                    mMoveZ += diff * Math.sin(mPitch);
                } else {
                    mMoveZ -= diff;
                    if(mMoveZ < 0)
                        mMoveZ = 0;
                    if(mMoveZ > 10)
                        mMoveZ = 10;
                }
                JNI.setView(mYawM + mYawR, mPitch, mMoveX, mMoveY, mMoveZ, mOrbit, !mViewMode);
            }

            private void Move(float f, float dx, float dy, float angle, float pitch) {
                //yaw rotation
                double fx = dx * f * Math.cos(angle) + dy * f * Math.sin(angle);
                double fy = dx * f * Math.sin(angle) - dy * f * Math.cos(angle);
                //pitch rotation
                mMoveX += dx * f * Math.cos(angle) * Math.cos(mPitch) - fx * Math.sin(mPitch);
                mMoveY += dx * f * Math.sin(angle) * Math.cos(mPitch) - fy * Math.sin(mPitch);
                mMoveZ += dy * f * Math.cos(mPitch);
            }
        }, mActivity);
    }


    private Bitmap createBitmapFromGLSurface(int x, int y, int w, int h, GL10 gl) {
        int bitmapBuffer[] = new int[w * h];
        int bitmapSource[] = new int[w * h];
        IntBuffer intBuffer = IntBuffer.wrap(bitmapBuffer);
        intBuffer.position(0);

        try {
            gl.glReadPixels(x, y, w, h, GL10.GL_RGBA, GL10.GL_UNSIGNED_BYTE, intBuffer);
            int offset1, offset2;
            for (int i = 0; i < h; i++) {
                offset1 = i * w;
                offset2 = (h - i - 1) * w;
                for (int j = 0; j < w; j++) {
                    int texturePixel = bitmapBuffer[offset1 + j];
                    int blue = (texturePixel >> 16) & 0xff;
                    int red = (texturePixel << 16) & 0x00ff0000;
                    int pixel = (texturePixel & 0xff00ff00) | red | blue;
                    bitmapSource[offset2 + j] = pixel;
                }
            }
        } catch (Exception e) {
            return null;
        }
        return Bitmap.createBitmap(bitmapSource, w, h, Bitmap.Config.ARGB_8888);
    }

    public void captureBitmap(boolean forced, String objFile)
    {
        final File thumbFile = new File(new File(objFile).getParent(), Exporter.getMtlResource(objFile) + ".png");
        if (forced || !thumbFile.exists() || thumbFile.length() < 1024)
        {
            try {
                while(!JNI.animFinished())
                {
                    Thread.sleep(20);
                }
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            synchronized(this) {
                mMakeThumbnail = thumbFile;
                mShareThumbnail = forced;
            }
            try {
                while(mMakeThumbnail != null)
                {
                    Thread.sleep(20);
                }
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }


    public void enterVR(String filename) {
        mPrevious = mView;
        updateView(ViewMode.VR);

        new Thread(() -> mActivity.runOnUiThread(() -> {
            Intent i;
            if (Compatibility.isDaydreamSupported(mActivity))
                i = new Intent(mActivity, DaydreamActivity.class);
            else
                i = new Intent(mActivity, CardboardActivity.class);
            i.setDataAndType(Uri.parse(filename), "text/plain");
            mActivity.startActivity(i);
        })).start();
    }

    private float getMoveFactor() {
        Display display = mActivity.getWindowManager().getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);
        return 2.0f / (size.x + size.y);
    }

    public ViewMode getViewMode() {
        return mView;
    }

    public View getVRButton() {
        return mCardboardButton;
    }

    public boolean isViewMode() {
        return mViewMode;
    }

    public void onDoubleClick(float move) {
        if (move < 0.05f) {
            int diff = 2;
            if (mView == ViewMode.FACE) diff = 1;
            if (mView == ViewMode.FIRST) diff = 5;
            mGestureDetector.listener().OnPinchToZoom(diff);
        }
    }

    public void restoreView() {
        updateView(mPrevious);
    }

    public void setOffset(float offset) {
        mMoveZ = offset;
    }

    public void setViewerMode(boolean face, boolean floorplan) {

        mFirstViewButton.setOnClickListener(view -> {
            mDistance.reset();
            if (mEditor.initialized())
                if (mEditor.movingLocked())
                    return;
            updateView(CameraControl.ViewMode.FIRST);
        });

        mOrbitViewButton.setOnClickListener(view -> {
            mDistance.reset();
            if (mEditor.initialized())
                if (mEditor.movingLocked())
                    return;
            updateView(CameraControl.ViewMode.ORBIT);
        });

        mTopViewButton.setOnClickListener(view -> {
            mDistance.reset();
            if (mEditor.initialized())
                if (mEditor.movingLocked())
                    return;
            updateView(CameraControl.ViewMode.TOPDOWN);
        });

        mMoveX = 0;
        mMoveY = 0;
        mMoveZ = 0;
        mPitch = 0;
        mYawM  = 0;
        mYawR  = 0;
        mViewMode = true;
        if (face) {
            updateView(CameraControl.ViewMode.FACE);
        } else if (floorplan) {
            updateView(CameraControl.ViewMode.FLOORPLAN);
        } else {
            updateView(CameraControl.ViewMode.TOPDOWN);
            updateView(CameraControl.ViewMode.ORBIT);
        }
    }

    public void updateButtons() {
        if (mEditor.initialized()) {
            final int visibility = mTopViewButton.getVisibility();
            final int newVisibility = mEditor.movingLocked() ? View.GONE : View.VISIBLE;
            if (visibility != newVisibility) {
                mActivity.runOnUiThread(() -> {
                    if (mView != ViewMode.FACE) {
                        mFirstViewButton.setVisibility(newVisibility);
                        mOrbitViewButton.setVisibility(newVisibility);
                        mTopViewButton.setVisibility(newVisibility);
                    }
                });
            }
        }
    }

    public void updateCapture(GL10 gl, GLESSurfaceView view) {

        if (mMakeThumbnail != null) {
            int size = Math.min(view.getWidth(), view.getHeight());
            int x = (view.getWidth() - size) / 2;
            int y = (view.getHeight() - size) / 2;
            Bitmap bitmap = createBitmapFromGLSurface(x, y, size, size, gl);
            if (bitmap != null) {
                try {
                    File temp = new File(AbstractActivity.getTempPath(), "thumb.png");
                    FileOutputStream out = new FileOutputStream(temp);
                    bitmap = Bitmap.createScaledBitmap(bitmap, 256, 256, false);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                    if (temp.renameTo(mMakeThumbnail)) {
                        File thumb = new File(mMakeThumbnail.getParent(), "thumbnail.jpg");
                        if (thumb.exists()) {
                            thumb.delete();
                        }
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            if (mShareThumbnail) {
                mShareThumbnail = false;
                int w = view.getWidth();
                int h = view.getHeight();
                bitmap = createBitmapFromGLSurface(0, 0, w, h, gl);
                if (bitmap != null) {
                    try {
                        File file = new File(AbstractActivity.getTempPath(), "3DLiveScanner.png");
                        FileOutputStream out = new FileOutputStream(file);
                        bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                        Intent intent = new Intent(Intent.ACTION_SEND);
                        intent.setType("image/png");
                        intent.putExtra(Intent.EXTRA_STREAM, FileProvider.getUriForFile(mActivity, BuildConfig.APPLICATION_ID + ".provider", file));
                        mActivity.startActivity(Intent.createChooser(intent, mActivity.getString(com.lvonasek.arcore3dscanner.R.string.share_via)));
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
            mMakeThumbnail = null;
        }
    }

    public void updateMotion(MotionEvent event) {
        try {
            mGestureDetector.onTouchEvent(event);
        } catch(Exception e) {
            e.printStackTrace();
        }

        //distance measuring gesture
        if (mViewMode) {
            int x1 = 0, y1 = 0, x2 = 0, y2 = 0, pointerCount = 0;
            for ( int i = 0; i < event.getPointerCount(); i++ )
            {
                pointerCount++;
                MotionEvent.PointerCoords pointerCoords = new MotionEvent.PointerCoords();
                event.getPointerCoords(i, pointerCoords);
                if( pointerCount == 1 )
                {
                    x1 = (int) pointerCoords.x;
                    y1 = (int) pointerCoords.y;
                }
                if( pointerCount == 2 )
                {
                    x2 = (int) pointerCoords.x;
                    y2 = (int) pointerCoords.y;
                }
                if( (event.getActionMasked() == MotionEvent.ACTION_POINTER_UP)
                        || (event.getActionMasked() == MotionEvent.ACTION_UP) )
                {
                    if( event.getActionIndex() == event.findPointerIndex(event.getPointerId(i)) )
                    {
                        pointerCount--;
                    }
                }
            }
            mDistance.setGesture(pointerCount, x1, y1, x2, y2);
            if ((mDiff > 0.1f) && (pointerCount == 1)) {
                mDistance.reset();
                mDiff = 0;
            }
        }
    }

    public void updateView() {
        JNI.setView(0, 0, 0, 0, mMoveZ, mOrbit, true);
    }

    public void updateView(ViewMode v) {
        mView = v;
        mCardboardButton.setBackgroundColor(mActivity.getColor(android.R.color.transparent));
        mFirstViewButton.setBackgroundColor(mActivity.getColor(android.R.color.transparent));
        mOrbitViewButton.setBackgroundColor(mActivity.getColor(android.R.color.transparent));
        mTopViewButton.setBackgroundColor(mActivity.getColor(android.R.color.transparent));

        float floor = JNI.getFloorLevel(mMoveX, mMoveY, mMoveZ);
        if (floor < -9999)
            floor = 0;
        switch (v) {
            case FACE:
                mOrbit = 0.5f;
                break;
            case FIRST:
                mOrbit = -1;
                mMoveZ = floor + 1.7f; //1.7m as an average human height
                mPitch = 0;
                mFirstViewButton.setBackgroundColor(mActivity.getColor(android.R.color.holo_blue_bright));
                break;
            case ORBIT:
                mOrbit = Math.max(5, Math.abs(mMoveZ - floor));
                mMoveZ = floor + 0.5f; //0.5m as an average object size
                mOrbitViewButton.setBackgroundColor(mActivity.getColor(android.R.color.holo_blue_bright));
                break;
            case TOPDOWN:
                mOrbit = -1;
                mMoveZ = floor + 10.0f;
                mPitch = (float) Math.toRadians(-90);
                mTopViewButton.setBackgroundColor(mActivity.getColor(android.R.color.holo_blue_bright));
                break;
            case VR:
                mOrbit = -1;
                mCardboardButton.setBackgroundColor(mActivity.getColor(android.R.color.holo_blue_bright));
                break;
            case FLOORPLAN:
                mOrbit = -1;
                mMoveZ = 10.0f;
                mPitch = (float) Math.toRadians(-90);
                break;
        }
        JNI.setView(mYawM + mYawR, mPitch, mMoveX, mMoveY, mMoveZ, mOrbit, false);
    }

    public void updateViewYaw(float offset) {
        offset = (float) Math.toRadians(offset);
        JNI.setView(mYawM + mYawR - offset, mPitch, mMoveX, mMoveY, mMoveZ, mOrbit, !mViewMode);
    }
}
