package com.lvonasek.tofviewer;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.graphics.Point;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.opengl.GLES20;
import android.opengl.Matrix;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Size;
import android.view.Surface;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.Collections;

/** Renders a point cloud. */
class DepthmapRenderer implements ImageReader.OnImageAvailableListener {
  private static final String TAG = TofViewerActivity.class.getSimpleName();

  // Shader names.
  private static final String VERTEX_SHADER_NAME = "shaders/depthmap.vert";
  private static final String FRAGMENT_SHADER_NAME = "shaders/depthmap.frag";

  private static final int BYTES_PER_FLOAT = Float.SIZE / 8;
  private static final int FLOATS_PER_POINT = 4; // X,Y,Z,C
  private static final int BYTES_PER_POINT = BYTES_PER_FLOAT * FLOATS_PER_POINT;

  private String depthCameraId = "0";
  private float cx = 0.5f;
  private float cy = 0.5f;
  private float fx = 1.0f;
  private float fy = 1.0f;
  private int depthWidth = 240;
  private int depthHeight = 180;
  private int programName;
  private int positionAttribute;
  private int pointSizeUniform;
  private int colorSchemeUniform;
  private int screenOrientationUniform;

  private int colorScheme = 0;
  private int numPoints = 0;
  private float[] array = null;
  private float[] points = null;
  private float[] depth = null;
  private float[] oldDepth = null;
  private float[] confidence = null;
  private boolean frameProcessing = false;
  private FloatBuffer verticesBuffer = null;

  private ImageReader mImageReader;
  private CameraDevice mCameraDevice;
  private float[] mAngles;

  /**
   * Allocates and initializes OpenGL resources needed by the plane renderer.
   *
   * @param context Needed to access shader source.
   */
  void createOnGlThread(Context context) throws IOException {
    ShaderUtil.checkGLError(TAG, "buffer alloc");

    int vertexShader = ShaderUtil.loadGLShader(TAG, context, GLES20.GL_VERTEX_SHADER, VERTEX_SHADER_NAME);
    int passthroughShader = ShaderUtil.loadGLShader(TAG, context, GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER_NAME);

    programName = GLES20.glCreateProgram();
    GLES20.glAttachShader(programName, vertexShader);
    GLES20.glAttachShader(programName, passthroughShader);
    GLES20.glLinkProgram(programName);
    GLES20.glUseProgram(programName);

    ShaderUtil.checkGLError(TAG, "program");

    positionAttribute = GLES20.glGetAttribLocation(programName, "a_Position");
    pointSizeUniform = GLES20.glGetUniformLocation(programName, "u_PointSize");
    screenOrientationUniform = GLES20.glGetUniformLocation(programName, "u_Rotation");
    colorSchemeUniform = GLES20.glGetUniformLocation(programName, "u_Scheme");
    ShaderUtil.checkGLError(TAG, "program params");
  }

  void initCamera(Context context) {
    depthWidth = 240;
    depthHeight = 180;
    String device = Build.MANUFACTURER.toUpperCase();

    try {
      CameraManager manager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
      for (String cameraId : manager.getCameraIdList()) {
        CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
        Integer facing = characteristics.get(CameraCharacteristics.LENS_FACING);
        if (facing != null) {
          if (facing == CameraCharacteristics.LENS_FACING_BACK) {
            int[] ch = characteristics.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES);
            if (ch != null) {
              for (int c : ch) {
                if (c == CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_DEPTH_OUTPUT) {
                  depthCameraId = cameraId;

                  if (device.startsWith("HONOR") || device.startsWith("HUAWEI")) {
                    StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                    for (Size size : map.getOutputSizes(ImageFormat.DEPTH16)) {
                      if (size.getWidth() == 224) {
                        depthWidth = size.getWidth();
                        depthHeight = size.getHeight();
                      }
                    }
                  }
                  break;
                }
              }
            }
          }
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
    }

    if (device.startsWith("LG")) {
      depthWidth = 224;
      depthHeight = 168;
    } else if (device.startsWith("SAMSUNG")) {
      //all supported Samsung devices except S10 5G have VGA resolution
      if (!Build.MODEL.startsWith("beyondx")) {
        depthWidth = 320;
        depthHeight = 240;
      }
    }
    cx *= depthWidth;
    cy *= depthHeight;
    fx *= depthWidth;
    fy *= depthHeight;
  }


  /**
   * Opens the camera
   */
  public void openCamera(Activity context) {
    if (context.checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
      return;
    }

    mImageReader = ImageReader.newInstance(depthWidth, depthHeight,
            ImageFormat.DEPTH16, 5);
    mImageReader.setOnImageAvailableListener(this, null);
    CameraManager manager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
    try {
      manager.openCamera(depthCameraId, callBack, null);
    } catch (CameraAccessException e) {
      e.printStackTrace();
    } catch (Exception e) {
      throw new RuntimeException("Interrupted while trying to lock camera opening.", e);
    }
  }

  /**
   * Closes the current {@link CameraDevice}.
   */
  public void closeCamera() {
    if (null != mImageReader) {
      mImageReader.close();
      mImageReader = null;
    }
    if (null != mCameraDevice)
    {
      mCameraDevice.close();
      mCameraDevice = null;
    }
  }

  // CPU image reader callback.
  @Override
  public void onImageAvailable(ImageReader imageReader) {
    synchronized (mCameraDevice) {
      if (frameProcessing) {
        return;
      }
      frameProcessing = true;
    }

    Image image = imageReader.acquireLatestImage();
    if (image == null) {
      synchronized (mCameraDevice) {
        frameProcessing = false;
      }
      return;
    }

    Image.Plane plane = image.getPlanes()[0];
    int stride = plane.getRowStride();
    ShortBuffer shortDepthBuffer = plane.getBuffer().asShortBuffer();
    if (confidence == null) confidence = new float[stride / 2 * image.getHeight()];
    if (oldDepth == null) oldDepth = new float[stride / 2 * image.getHeight()];
    if (depth == null) depth = new float[stride / 2 * image.getHeight()];
    float[] temp = oldDepth;
    oldDepth = depth;
    depth = temp;

    int count = 0;
    int index = 0;
    float diff = 0;
    float avgDepth = 0;
    while (shortDepthBuffer.hasRemaining()) {
      int depthSample = shortDepthBuffer.get();
      int depthConfidence = ((depthSample >> 13) & 0x7);
      confidence[index] = depthConfidence == 0 ? 1.f : (depthConfidence - 1) / 7.f;

      float depthValue = (depthSample & 0x1FFF) * 0.001f;
      if (depthValue < 3) {
        diff += Math.abs(oldDepth[index] - depthValue) > 0.03f ? 1 : 0;
        count++;
      }
      depth[index] = depthValue;
      avgDepth += depthValue;
      index++;
    }
    depthWidth = image.getWidth();
    depthHeight = image.getHeight();
    int size = depthWidth * depthHeight;
    image.close();

    if (index > 0) {
      avgDepth /= index;
      if (avgDepth > 0.25f) {
        if (count > 0) {
          diff /= count;
          diff = diff * diff * 100.0f;
          float lerp = Math.max(Math.min(diff, 1.0f), 0.25f);
          for (int i = 0; i < index; i++) {
            if ((depth[i] > 0) && (oldDepth[i] > 0)) {
              depth[i] = oldDepth[i] * (1 - lerp) + depth[i] * lerp;
            }
          }
        }
      }
    }

    Log.d("TofViewer", "noise_filter_factor=" + diff + ", average_depth=" + avgDepth);

    //prepare data for render
    synchronized (this) {
      numPoints = 0;
      if (array == null) array = new float[size * FLOATS_PER_POINT];
      if (points == null) points = new float[size * FLOATS_PER_POINT];
      boolean normalMode = colorScheme == 4;
      if (normalMode) {
        createPointcloud(stride);
        createNormalmap();
      } else {
        createDepthmap(stride);
      }

      if (verticesBuffer == null) {
        ByteBuffer buffer = ByteBuffer.allocateDirect(size * BYTES_PER_POINT);
        buffer.order(ByteOrder.nativeOrder());
        verticesBuffer = buffer.asFloatBuffer();
      }
      verticesBuffer.rewind();
      verticesBuffer.put(array);
      verticesBuffer.position(0);

      synchronized (mCameraDevice) {
        frameProcessing = false;
      }
    }
  }

  CameraDevice.StateCallback callBack = new CameraDevice.StateCallback() {
    @Override
    public void onOpened(CameraDevice cameraDevice) {
      Surface imageReaderSurface = mImageReader.getSurface();
      mCameraDevice = cameraDevice;

      try {
        final CaptureRequest.Builder requestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
        requestBuilder.addTarget(imageReaderSurface);

        cameraDevice.createCaptureSession(Collections.singletonList(imageReaderSurface),new CameraCaptureSession.StateCallback() {

          @Override
          public void onConfigured(CameraCaptureSession cameraCaptureSession) {
            CameraCaptureSession.CaptureCallback captureCallback = new CameraCaptureSession.CaptureCallback() {};

            try {
              HandlerThread handlerThread = new HandlerThread("DepthBackgroundThread");
              handlerThread.start();
              Handler handler = new Handler(handlerThread.getLooper());
              cameraCaptureSession.setRepeatingRequest(requestBuilder.build(),captureCallback,handler);

            } catch (CameraAccessException e) {
              e.printStackTrace();
            }
          }
          @Override
          public void onConfigureFailed(CameraCaptureSession cameraCaptureSession) {

          }
        },null);
      } catch (CameraAccessException e) {
        e.printStackTrace();
      }
    }
    @Override
    public void onDisconnected(CameraDevice cameraDevice) {
    }
    @Override
    public void onError(CameraDevice cameraDevice, int i) {
    }
  };

  /**
   * Renders the point cloud. ARCore point cloud is given in world space.
   */
  synchronized void draw(Activity context, float zoom) {

    if (verticesBuffer == null)
      return;

    ShaderUtil.checkGLError(TAG, "Before draw");

    Point size = new Point();
    context.getWindowManager().getDefaultDisplay().getSize(size);
    float x = size.x / (float)depthWidth;
    float y = size.y / (float)depthHeight;

    GLES20.glClearColor(0f, 0f, 0f, 1.0f);
    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
    GLES20.glUseProgram(programName);
    GLES20.glEnableVertexAttribArray(positionAttribute);
    GLES20.glVertexAttribPointer(positionAttribute, FLOATS_PER_POINT, GLES20.GL_FLOAT, false, BYTES_PER_POINT, verticesBuffer);
    GLES20.glUniform1f(pointSizeUniform, Math.max(x, y) * getStep() * zoom);

    int rotation = context.getWindowManager().getDefaultDisplay().getRotation();
    switch (rotation) {
      case Surface.ROTATION_0:
        GLES20.glUniform1f(screenOrientationUniform, 0);
        break;
      case Surface.ROTATION_90:
        GLES20.glUniform1f(screenOrientationUniform, -90);
        break;
      case Surface.ROTATION_180:
        GLES20.glUniform1f(screenOrientationUniform, 180);
        break;
      case Surface.ROTATION_270:
        GLES20.glUniform1f(screenOrientationUniform, 90);
        break;
    }
    GLES20.glUniform1f(colorSchemeUniform, (float)colorScheme);

    GLES20.glDrawArrays(GLES20.GL_POINTS, 0, numPoints);
    GLES20.glDisableVertexAttribArray(positionAttribute);
    ShaderUtil.checkGLError(TAG, "Draw");
  }

  public void setColorScheme(int scheme) {
    colorScheme = scheme;
  }

  private void createDepthmap(int stride) {
    int index = 0;
    for (int y = 0; y < depthHeight; y++) {
      for (int x = 0; x < depthWidth; x++) {
        if (depth[y * stride / 2 + x] > 0) {
          array[index + 0] = 2.0f * (x + 0.5f) / (float)depthWidth - 1.0f;
          array[index + 1] = -2.0f * (y + 0.5f) / (float)depthHeight + 1.0f;
          array[index + 2] = depth[y * stride / 2 + x];
          array[index + 3] = confidence[y * stride / 2 + x];
          numPoints++;
          index += FLOATS_PER_POINT;
        }
      }
    }
  }

  private void createNormalmap() {
    int index = 0;
    float[] p1 = new float[3];
    float[] p2 = new float[3];
    float[] result = new float[3];
    int step = getStep();
    for (int y = step; y < depthHeight; y += step) {
      for (int x = step; x < depthWidth; x += step) {
        int i = (y * depthWidth + x) * FLOATS_PER_POINT;
        if (points[i + 3] > 0.5f) {
          for (int j = 0; j < 3; j++) {
            p1[j] = points[i + j] - points[i - FLOATS_PER_POINT * step + j];
            p2[j] = points[i + j] - points[i - depthWidth * FLOATS_PER_POINT * step + j];
          }
          result[0] = p1[1] * p2[2] - p2[1] * p1[2];
          result[1] = p1[2] * p2[0] - p2[2] * p1[0];
          result[2] = p1[0] * p2[1] - p2[0] * p1[1];

          float len = Math.abs(result[0]) + Math.abs(result[1]) + Math.abs(result[2]);
          if (len == 0) {
            len = 1;
          }
          array[index + 0] = 2.0f * (x + 0.5f) / (float)depthWidth - 1.0f;
          array[index + 1] = -2.0f * (y + 0.5f) / (float)depthHeight + 1.0f;
          array[index + 2] = Math.abs(result[0]) / len;
          array[index + 3] = Math.abs(result[1]) / len;
          index += FLOATS_PER_POINT;
          numPoints++;
        }
      }
    }
  }

  private void createPointcloud(int stride) {
    float[] matrix = new float[16];
    float[] rotate = new float[16];
    float pitch = (float) Math.toDegrees(mAngles[0]);
    float roll = (float) Math.toDegrees(mAngles[2]);
    Matrix.setRotateEulerM(matrix, 0, 0, 0, roll);
    Matrix.setRotateEulerM(rotate, 0, -pitch, 0, 0);
    Matrix.multiplyMM(matrix, 0, rotate, 0, matrix, 0);

    int step = getStep();
    for (int y = 0; y < depthHeight; y += step) {
      for (int x = 0; x < depthWidth; x += step) {
        int index = (y * depthWidth + x) * FLOATS_PER_POINT;
        float depthValue = depth[y * stride / 2 + x];
        if (depthValue > 0) {
          float[] p = {
                  (x - cx) * depthValue / fx,
                  (y - cy) * depthValue / fy,
                  depthValue,
                  1
          };
          Matrix.multiplyMV(p, 0, matrix, 0, p, 0);
          for (int i = 0; i < 3; i++) {
            points[index + i] = p[i] / Math.abs(p[3]);
          }
        }
        points[index + 3] = depthValue > 0 ? 1 : 0;
      }
    }
  }

  private int getStep() {
    if ((colorScheme == 4) && (depthWidth > 250)) {
      return 2;
    } else {
      return 1;
    }
  }

  public void setAngles(float[] value) {
    mAngles = value;
  }
}
