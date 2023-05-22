package com.lvonasek.gles;

import java.lang.ref.WeakReference;
import java.util.ArrayList;

import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGL11;
import javax.microedition.khronos.opengles.GL10;

class GLESThread extends Thread
{
  private final GLESThreadManager       sGLThreadManager = new GLESThreadManager();

  // Once the thread is started, all accesses to the following member
  // variables are protected by the sGLThreadManager monitor
  private boolean                        mShouldExit;
  private boolean                        mExited;
  private boolean                        mHasSurface;
  private boolean                        mSurfaceIsBad;
  private boolean                        mWaitingForSurface;
  private boolean                        mHaveEglContext;
  private boolean                        mHaveEglSurface;
  private boolean                        mFinishedCreatingEglSurface;
  private boolean                        mShouldReleaseEglContext;
  private int                            mWidth;
  private int                            mHeight;
  private boolean                        mRenderComplete;
  private ArrayList<Runnable> mEventQueue  = new ArrayList<>();
  private boolean                        mSizeChanged = true;

  // End of member variables protected by the sGLThreadManager monitor.
  private EGLHelper                      mEglHelper;
  private WeakReference<GLESSurfaceView> mGLESSurfaceViewWeakRef;

  GLESThread(WeakReference<GLESSurfaceView> GLESSurfaceViewWeakRef)
  {
    super();
    mWidth = 0;
    mHeight = 0;
    mGLESSurfaceViewWeakRef = GLESSurfaceViewWeakRef;
  }

  @Override
  public void run()
  {
    setName("MF GLThread " + getId());

    try
    {
      guardedRun();
    } catch (InterruptedException e)
    {
      e.printStackTrace();
    } finally
    {
      mExited = true;
      sGLThreadManager.threadExiting(this);
    }
  }

  private void stopEglSurfaceLocked()
  {
    if (mHaveEglSurface)
    {
      mHaveEglSurface = false;
      mEglHelper.destroySurface();
    }
  }

  private void stopEglContextLocked()
  {
    if (mHaveEglContext)
    {
      mEglHelper.finish();
      mHaveEglContext = false;
      sGLThreadManager.releaseEglContextLocked(this);
    }
  }

  private void guardedRun() throws InterruptedException
  {
    mEglHelper = new EGLHelper(mGLESSurfaceViewWeakRef);
    mHaveEglContext = false;
    mHaveEglSurface = false;
    try
    {
      GL10 gl = null;
      boolean createEglContext = false;
      boolean createEglSurface = false;
      boolean createGlInterface = false;
      boolean lostEglContext = false;
      boolean sizeChanged = false;
      boolean wantRenderNotification = false;
      boolean doRenderNotification = false;
      boolean askedToReleaseEglContext = false;
      int w = 0;
      int h = 0;
      Runnable event = null;

      while (true)
      {
        synchronized (sGLThreadManager)
        {
          while (true)
          {
            if (mShouldExit)
              return;

            if (!mEventQueue.isEmpty())
            {
              event = mEventQueue.remove(0);
              break;
            }

            // Do we need to give up the EGL context?
            if (mShouldReleaseEglContext)
            {
              stopEglSurfaceLocked();
              stopEglContextLocked();
              mShouldReleaseEglContext = false;
              askedToReleaseEglContext = true;
            }

            // Have we lost the EGL context?
            if (lostEglContext)
            {
              stopEglSurfaceLocked();
              stopEglContextLocked();
              lostEglContext = false;
            }

            // Have we lost the SurfaceView surface?
            if ((!mHasSurface) && (!mWaitingForSurface))
            {
              if (mHaveEglSurface)
                stopEglSurfaceLocked();
              mWaitingForSurface = true;
              mSurfaceIsBad = false;
              sGLThreadManager.notifyAll();
            }

            // Have we acquired the surface view surface?
            if (mHasSurface && mWaitingForSurface)
            {
              mWaitingForSurface = false;
              sGLThreadManager.notifyAll();
            }

            if (doRenderNotification)
            {
              wantRenderNotification = false;
              doRenderNotification = false;
              mRenderComplete = true;
              sGLThreadManager.notifyAll();
            }

            // Ready to draw?
            if (readyToDraw())
            {

              // If we don't have an EGL context, try to acquire one.
              if (!mHaveEglContext)
              {
                if (askedToReleaseEglContext)
                  askedToReleaseEglContext = false;
                else if (sGLThreadManager.tryAcquireEglContextLocked(this))
                {
                  try
                  {
                    mEglHelper.start();
                  } catch (RuntimeException t)
                  {
                    sGLThreadManager.releaseEglContextLocked(this);
                    throw t;
                  }
                  mHaveEglContext = true;
                  createEglContext = true;

                  sGLThreadManager.notifyAll();
                }
              }

              if (mHaveEglContext && !mHaveEglSurface)
              {
                mHaveEglSurface = true;
                createEglSurface = true;
                createGlInterface = true;
                sizeChanged = true;
              }

              if (mHaveEglSurface)
              {
                if (mSizeChanged)
                {
                  sizeChanged = true;
                  w = mWidth;
                  h = mHeight;
                  wantRenderNotification = true;

                  // Destroy and recreate the EGL surface.
                  createEglSurface = true;
                  mSizeChanged = false;
                }
                sGLThreadManager.notifyAll();
                break;
              }
            }
            sGLThreadManager.wait();
          }
        } // end of synchronized(sGLThreadManager)

        if (event != null)
        {
          event.run();
          event = null;
          continue;
        }

        if (createEglSurface)
        {
          if (mEglHelper.createSurface())
          {
            synchronized (sGLThreadManager)
            {
              mFinishedCreatingEglSurface = true;
              sGLThreadManager.notifyAll();
            }
          } else
          {
            synchronized (sGLThreadManager)
            {
              mFinishedCreatingEglSurface = true;
              mSurfaceIsBad = true;
              sGLThreadManager.notifyAll();
            }
            continue;
          }
          createEglSurface = false;
        }

        if (createGlInterface)
        {
          gl = (GL10) mEglHelper.createGL();
          sGLThreadManager.checkGLDriver(gl);
          createGlInterface = false;
        }

        if (createEglContext)
        {
          GLESSurfaceView view = mGLESSurfaceViewWeakRef.get();
          if (view != null)
            if (view.mRenderer != null)
            {
              view.mRenderer.onSurfaceCreated(gl, mEglHelper.getConfig());
              createEglContext = false;
            }
        }

        if (sizeChanged)
        {
          GLESSurfaceView view = mGLESSurfaceViewWeakRef.get();
          if (view != null)
            if (view.mRenderer != null)
            {
              view.mRenderer.onSurfaceChanged(gl, w, h);
              sizeChanged = false;
            }
        }

        {
          GLESSurfaceView view = mGLESSurfaceViewWeakRef.get();
          if (view != null)
            if (view.mRenderer != null)
              view.mRenderer.onDrawFrame(gl);
        }
        int swapError = mEglHelper.swap();
        switch (swapError)
        {
          case EGL10.EGL_SUCCESS:
            break;
          case EGL11.EGL_CONTEXT_LOST:
            lostEglContext = true;
            break;
          default:
            synchronized (sGLThreadManager)
            {
              mSurfaceIsBad = true;
              sGLThreadManager.notifyAll();
            }
            break;
        }
        if (wantRenderNotification)
          doRenderNotification = true;
      }

    } catch(Exception e)
    {
      synchronized (sGLThreadManager)
      {
        stopEglSurfaceLocked();
        stopEglContextLocked();
      }
    }
  }

  private boolean ableToDraw()
  {
    return mHaveEglContext && mHaveEglSurface && readyToDraw();
  }

  private boolean readyToDraw()
  {
    return mHasSurface && (!mSurfaceIsBad) && (mWidth > 0) && (mHeight > 0);
  }

  void surfaceCreated()
  {
    synchronized (sGLThreadManager)
    {
      mHasSurface = true;
      mFinishedCreatingEglSurface = false;
      sGLThreadManager.notifyAll();
      while (mWaitingForSurface && !mFinishedCreatingEglSurface && !mExited)
      {
        sleep();
      }
    }
  }

  void surfaceDestroyed()
  {
    synchronized (sGLThreadManager)
    {
      mHasSurface = false;
      sGLThreadManager.notifyAll();
      while ((!mWaitingForSurface) && (!mExited))
      {
        sleep();
      }
    }
  }

  void onWindowResize(int w, int h)
  {
    synchronized (sGLThreadManager)
    {
      mWidth = w;
      mHeight = h;
      mSizeChanged = true;
      mRenderComplete = false;
      sGLThreadManager.notifyAll();

      // Wait for thread to react to resize and render a frame
      while (!mExited && !mRenderComplete && ableToDraw())
      {
        sleep();
      }
    }
  }

  void requestExitAndWait()
  {
    // don't call this from GLThread thread or it is a guaranteed
    // deadlock!
    synchronized (sGLThreadManager)
    {
      mShouldExit = true;
      sGLThreadManager.notifyAll();
      while (!mExited)
      {
        sleep();
      }
    }
  }

  void requestReleaseEglContextLocked()
  {
    mShouldReleaseEglContext = true;
    sGLThreadManager.notifyAll();
  }

  void queueEvent(Runnable r)
  {
    if (r == null)
      throw new IllegalArgumentException("r must not be null");
    synchronized (sGLThreadManager)
    {
      mEventQueue.add(r);
      sGLThreadManager.notifyAll();
    }
  }

  private void sleep()
  {
    try
    {
      Thread.sleep(5);
      sGLThreadManager.wait();
    } catch (InterruptedException e)
    {
      e.printStackTrace();
    }
  }
}
