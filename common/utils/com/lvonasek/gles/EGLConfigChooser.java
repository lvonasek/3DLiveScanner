package com.lvonasek.gles;

import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.egl.EGLDisplay;

class EGLConfigChooser
{
  private int[] mConfigSpec;
  private int[] mValue;
  // Subclasses can adjust these values:
  private int mRedSize;
  private int mGreenSize;
  private int mBlueSize;
  private int mAlphaSize;
  private int mDepthSize;
  private int mStencilSize;

  EGLConfigChooser(int redSize, int greenSize, int blueSize, int alphaSize, int depthSize, int stencilSize)
  {
    mConfigSpec = filterConfigSpec(new int[] { EGL10.EGL_RED_SIZE, redSize, EGL10.EGL_GREEN_SIZE, greenSize, EGL10.EGL_BLUE_SIZE, blueSize,
            EGL10.EGL_ALPHA_SIZE, alphaSize, EGL10.EGL_DEPTH_SIZE, depthSize, EGL10.EGL_STENCIL_SIZE, stencilSize,
            EGL10.EGL_NONE });
    mValue = new int[1];
    mRedSize = redSize;
    mGreenSize = greenSize;
    mBlueSize = blueSize;
    mAlphaSize = alphaSize;
    mDepthSize = depthSize;
    mStencilSize = stencilSize;
  }

  EGLConfig chooseConfig(EGL10 egl, EGLDisplay display)
  {
    int[] num_config = new int[1];
    if (!egl.eglChooseConfig(display, mConfigSpec, null, 0, num_config))
      throw new IllegalArgumentException("eglChooseConfig failed");

    int numConfigs = num_config[0];
    if (numConfigs <= 0)
      throw new IllegalArgumentException("No configs match configSpec");

    EGLConfig[] configs = new EGLConfig[numConfigs];
    if (!egl.eglChooseConfig(display, mConfigSpec, configs, numConfigs, num_config))
      throw new IllegalArgumentException("eglChooseConfig#2 failed");
    EGLConfig config = chooseConfig(egl, display, configs);
    if (config == null)
      throw new IllegalArgumentException("No config chosen");
    return config;
  }

  private EGLConfig chooseConfig(EGL10 egl, EGLDisplay display, EGLConfig[] configs)
  {
    for (EGLConfig config : configs)
    {
      int d = findConfigAttribute(egl, display, config, EGL10.EGL_DEPTH_SIZE, 0);
      int s = findConfigAttribute(egl, display, config, EGL10.EGL_STENCIL_SIZE, 0);
      if ((d >= mDepthSize) && (s >= mStencilSize))
      {
        int r = findConfigAttribute(egl, display, config, EGL10.EGL_RED_SIZE, 0);
        int g = findConfigAttribute(egl, display, config, EGL10.EGL_GREEN_SIZE, 0);
        int b = findConfigAttribute(egl, display, config, EGL10.EGL_BLUE_SIZE, 0);
        int a = findConfigAttribute(egl, display, config, EGL10.EGL_ALPHA_SIZE, 0);
        if ((r == mRedSize) && (g == mGreenSize) && (b == mBlueSize) && (a == mAlphaSize))
          return config;
      }
    }
    return null;
  }

  private int[] filterConfigSpec(int[] configSpec)
  {
    int len = configSpec.length;
    int[] newConfigSpec = new int[len + 2];
    System.arraycopy(configSpec, 0, newConfigSpec, 0, len - 1);
    newConfigSpec[len - 1] = EGL10.EGL_RENDERABLE_TYPE;
    newConfigSpec[len] = 4; /* EGL_OPENGL_ES2_BIT */
    newConfigSpec[len + 1] = EGL10.EGL_NONE;
    return newConfigSpec;
  }

  private int findConfigAttribute(EGL10 egl, EGLDisplay display, EGLConfig config, int attribute, int defaultValue)
  {
    if (egl.eglGetConfigAttrib(display, config, attribute, mValue))
      return mValue[0];
    return defaultValue;
  }
}
