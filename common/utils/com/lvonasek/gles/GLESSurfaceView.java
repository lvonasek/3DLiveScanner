package com.lvonasek.gles;

import java.lang.ref.WeakReference;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.content.Context;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

public class GLESSurfaceView extends SurfaceView implements SurfaceHolder.Callback
{
  public GLESSurfaceView(Context context)
  {
    super(context);
    getHolder().addCallback(this);
  }

  public GLESSurfaceView(Context context, AttributeSet attrs)
  {
    super(context, attrs);
    getHolder().addCallback(this);
  }

  public void setRenderer(Renderer renderer)
  {
    if (mGLThread != null)
      throw new IllegalStateException("setRenderer has already been called for this instance.");
    mRenderer = renderer;
  }

  public void surfaceCreated(SurfaceHolder holder)
  {
    mGLThread.surfaceCreated();
  }

  public void surfaceDestroyed(SurfaceHolder holder)
  {
    mGLThread.surfaceDestroyed();
  }

  public void surfaceChanged(SurfaceHolder holder, int format, int w, int h)
  {
    mGLThread.onWindowResize(w, h);
  }

  public void stop()
  {
    if (mGLThread != null)
      mGLThread.requestExitAndWait();
  }

  @Override
  protected synchronized void onAttachedToWindow()
  {
    super.onAttachedToWindow();
    mGLThread = new GLESThread(mThisWeakRef);
    mGLThread.start();
  }

  @Override
  protected synchronized void onDetachedFromWindow()
  {
    stop();
    super.onDetachedFromWindow();
  }

  public interface Renderer
  {
    void onSurfaceCreated(GL10 gl, EGLConfig config);

    void onSurfaceChanged(GL10 gl, int width, int height);

    void onDrawFrame(GL10 gl);
  }

  private final WeakReference<GLESSurfaceView> mThisWeakRef     = new WeakReference<>(this);
  private GLESThread                           mGLThread;
  protected Renderer                           mRenderer;
}