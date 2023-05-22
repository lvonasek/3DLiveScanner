package com.lvonasek.arcore3dscanner.vr;

import android.opengl.GLES20;
import android.opengl.Matrix;
import android.os.Bundle;

import com.google.vr.sdk.base.AndroidCompat;
import com.google.vr.sdk.base.Eye;
import com.google.vr.sdk.base.GvrActivity;
import com.google.vr.sdk.base.GvrView;
import com.google.vr.sdk.base.HeadTransform;
import com.google.vr.sdk.base.Viewport;
import com.lvonasek.arcore3dscanner.R;

import javax.microedition.khronos.egl.EGLConfig;

public class VRActivity extends GvrActivity implements GvrView.StereoRenderer, Runnable
{
  static {
    System.loadLibrary("3dscanner");
  }

  protected boolean ready = false;
  protected static String filename;
  protected float[] headView = new float[16];
  protected float[] modelViewProjection = new float[16];
  protected float[] rotation = new float[16];
  protected float yaw = 0;

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.vr);

    //stereo view
    GvrView gvrView = (GvrView) findViewById(R.id.gvr_view);
    gvrView.setEGLConfigChooser(8, 8, 8, 8, 24, 8);
    gvrView.setEGLContextClientVersion(3);
    gvrView.setRenderer(this);
    gvrView.setTransitionViewEnabled(true);
    gvrView.setDistortionCorrectionEnabled(true);
    if (gvrView.setAsyncReprojectionEnabled(true))
      AndroidCompat.setSustainedPerformanceMode(this, true);
    setGvrView(gvrView);
    filename = getIntent().getDataString();
  }

  @Override
  public synchronized void onRendererShutdown() {
  }

  @Override
  public synchronized void onSurfaceChanged(int width, int height) {
  }

  @Override
  public synchronized void onSurfaceCreated(EGLConfig config) {
    GLES20.glClearColor(0, 0, 0, 1);
    nativeInitializeGl();
    new Thread(this).start();
  }

  @Override
  public synchronized void onNewFrame(HeadTransform headTransform) {
    Matrix.setRotateM(rotation, 0, yaw, 0, 1, 0);
    headTransform.getHeadView(headView, 0);
    Matrix.multiplyMM(headView, 0, headView, 0, rotation, 0);
  }

  @Override
  public synchronized void onDrawEye(Eye eye) {
    GLES20.glEnable(GLES20.GL_DEPTH_TEST);
    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
    float[] perspective = eye.getPerspective(0.1f, 1000.0f);
    float[] view = eye.getEyeView();
    Matrix.multiplyMM(view, 0, view, 0, rotation, 0);
    Matrix.multiplyMM(modelViewProjection, 0, perspective, 0, view, 0);
    nativeDrawFrame(modelViewProjection);
  }

  @Override
  public synchronized void onFinishFrame(Viewport viewport) {
    if (ready)
      nativeUpdate();
  }

  protected synchronized native void nativeLoadModel(String filename);
  protected synchronized native void nativeInitializeGl();
  protected synchronized native void nativeDrawFrame(float[] matrix);
  protected synchronized native void nativeOnTriggerEvent(float x, float y, float z, float[] matrix);
  protected synchronized native void nativeUpdate();

  @Override
  public void run() {
    nativeLoadModel(filename);
    ready = true;
  }
}
