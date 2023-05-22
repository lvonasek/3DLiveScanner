package com.lvonasek.gles;

import android.util.Log;

import java.lang.ref.WeakReference;

import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.egl.EGLContext;
import javax.microedition.khronos.egl.EGLDisplay;
import javax.microedition.khronos.egl.EGLSurface;
import javax.microedition.khronos.opengles.GL;

class EGLHelper
{
  private static final int EGL_CONTEXT_CLIENT_VERSION = 0x3098;
  private static EGLContext lastContext = null;

  private WeakReference<GLESSurfaceView>         mGLESSurfaceViewWeakRef;
  private EGL10                                  mEgl;
  private EGLDisplay                             mEglDisplay;
  private EGLSurface                             mEglSurface;
  private EGLConfig                              mEglConfig;
  private EGLContext                             mEglContext;

  EGLHelper(WeakReference<GLESSurfaceView> GLESSurfaceViewWeakRef)
  {
    mGLESSurfaceViewWeakRef = GLESSurfaceViewWeakRef;
  }

  public void start()
  {
    mEgl = (EGL10) EGLContext.getEGL();
    mEglDisplay = mEgl.eglGetDisplay(EGL10.EGL_DEFAULT_DISPLAY);
    if (mEglDisplay == EGL10.EGL_NO_DISPLAY)
      throw new RuntimeException("eglGetDisplay failed");

    int[] version = new int[2];
    if (!mEgl.eglInitialize(mEglDisplay, version))
      throw new RuntimeException("eglInitialize failed");
    GLESSurfaceView view = mGLESSurfaceViewWeakRef.get();
    if (view == null)
    {
      mEglConfig = null;
      mEglContext = null;
    } else
    {
      mEglConfig = new EGLConfigChooser(8, 8, 8, 8, 16, 8).chooseConfig(mEgl, mEglDisplay);
      if (lastContext == null)
      {
        int[] att = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL10.EGL_NONE };
        lastContext = mEgl.eglCreateContext(mEglDisplay, mEglConfig, EGL10.EGL_NO_CONTEXT, att);
      }
      mEglContext = lastContext;
    }
    if (mEglContext == null || mEglContext == EGL10.EGL_NO_CONTEXT)
    {
      mEglContext = null;
      throw new RuntimeException("createContext");
    }
    mEglSurface = null;
  }

  boolean createSurface()
  {
    if (mEgl == null)
      throw new RuntimeException("egl not initialized");
    if (mEglDisplay == null)
      throw new RuntimeException("eglDisplay not initialized");
    if (mEglConfig == null)
      throw new RuntimeException("mEglConfig not initialized");
    destroySurface();

    GLESSurfaceView view = mGLESSurfaceViewWeakRef.get();
    if (view != null)
    {
      try
      {
        mEglSurface = mEgl.eglCreateWindowSurface(mEglDisplay, mEglConfig, view.getHolder(), null);
      } catch (IllegalArgumentException e)
      {
        e.printStackTrace();
      }
    }
    else
      mEglSurface = null;

    if (mEglSurface == null || mEglSurface == EGL10.EGL_NO_SURFACE)
    {
      if (mEgl.eglGetError() == EGL10.EGL_BAD_NATIVE_WINDOW)
        Log.e("EglHelper", "createWindowSurface returned EGL_BAD_NATIVE_WINDOW.");
      return false;
    }
    return mEgl.eglMakeCurrent(mEglDisplay, mEglSurface, mEglSurface, mEglContext);
  }

  GL createGL()
  {
    return mEglContext.getGL();
  }

  int swap()
  {
    if (!mEgl.eglSwapBuffers(mEglDisplay, mEglSurface))
      return mEgl.eglGetError();
    return EGL10.EGL_SUCCESS;
  }

  void destroySurface()
  {
    if (mEglSurface != null && mEglSurface != EGL10.EGL_NO_SURFACE)
    {
      mEgl.eglMakeCurrent(mEglDisplay, EGL10.EGL_NO_SURFACE, EGL10.EGL_NO_SURFACE, EGL10.EGL_NO_CONTEXT);
      mEgl.eglDestroySurface(mEglDisplay, mEglSurface);
      mEglSurface = null;
    }
  }

  public void finish()
  {
    if (mEglContext != null)
      mEglContext = null;
    if (mEglDisplay != null)
    {
      mEgl.eglTerminate(mEglDisplay);
      mEglDisplay = null;
    }
  }

  public EGLConfig getConfig()
  {
    return mEglConfig;
  }
}
