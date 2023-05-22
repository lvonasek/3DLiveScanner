package com.lvonasek.gles;

import javax.microedition.khronos.opengles.GL10;

class GLESThreadManager
{
  private boolean             mGLESVersionCheckComplete;
  private int                 mGLESVersion;
  private boolean             mGLESDriverCheckComplete;
  private boolean             mMultipleGLESContextsAllowed;
  private static final int    GLES_20 = 0x20000;
  private static final String MSM7K_RENDERER_PREFIX = "Q3Dimension MSM7500 ";
  private GLESThread          mEglOwner;

  synchronized void threadExiting(GLESThread thread)
  {
    if (mEglOwner == thread)
      mEglOwner = null;
    notifyAll();
  }

  boolean tryAcquireEglContextLocked(GLESThread thread)
  {
    if (mEglOwner == thread || mEglOwner == null)
    {
      mEglOwner = thread;
      notifyAll();
      return true;
    }
    checkGLESVersion();
    if (mMultipleGLESContextsAllowed)
      return true;
    if (mEglOwner != null)
      mEglOwner.requestReleaseEglContextLocked();
    return false;
  }

  void releaseEglContextLocked(GLESThread thread)
  {
    if (mEglOwner == thread)
      mEglOwner = null;
    notifyAll();
  }

  synchronized void checkGLDriver(GL10 gl)
  {
    if (!mGLESDriverCheckComplete)
    {
      checkGLESVersion();
      String renderer = gl.glGetString(GL10.GL_RENDERER);
      if (mGLESVersion < GLES_20)
      {
        mMultipleGLESContextsAllowed = !renderer.startsWith(MSM7K_RENDERER_PREFIX);
        notifyAll();
      }
      mGLESDriverCheckComplete = true;
    }
  }

  private void checkGLESVersion()
  {
    if (!mGLESVersionCheckComplete)
    {
      mGLESVersion = GLES_20;
      mMultipleGLESContextsAllowed = true;
      mGLESVersionCheckComplete = true;
    }
  }
}
