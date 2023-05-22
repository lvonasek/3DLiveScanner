package com.lvonasek.tofviewer;

import android.Manifest;
import android.app.AlertDialog;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.Point;
import android.opengl.GLES20;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.provider.Settings;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;

import com.google.vr.sdk.base.AndroidCompat;
import com.google.vr.sdk.base.Eye;
import com.google.vr.sdk.base.GvrActivity;
import com.google.vr.sdk.base.GvrView;
import com.google.vr.sdk.base.HeadTransform;
import com.google.vr.sdk.base.Viewport;
import com.lvonasek.gles.GLESSurfaceView;
import com.lvonasek.record.Recorder;

import java.io.IOException;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class TofViewerActivity extends GvrActivity implements GLESSurfaceView.Renderer, View.OnTouchListener {

  private static final String KEY_EYE_DOWN = "KEY_EYE_DOWN";
  private static final String KEY_EYE_SIDE = "KEY_EYE_SIDE";
  private static final String KEY_EYE_ZOOM = "KEY_EYE_ZOOM";
  private static final String KEY_SCHEME = "KEY_SCHEME";

  private final String[] permissions = {
          Manifest.permission.CAMERA,
          Manifest.permission.RECORD_AUDIO,
          Manifest.permission.READ_EXTERNAL_STORAGE,
          Manifest.permission.WRITE_EXTERNAL_STORAGE
  };

  private static final int REQUEST_PERMISSIONS = 200;

  private final DepthmapRenderer depthmapRenderer = new DepthmapRenderer();

  private static boolean connected = false;
  private static boolean recordNight = false;
  private static boolean vrMode = false;

  private boolean initialized = false;
  private boolean mMakePhoto = false;

  private Button mMoreButton;
  private Button mRecordButton;
  private Button mThumbnailButton;
  private LinearLayout mVRSetup;
  private GLESSurfaceView mSurfaceView;
  private ScaleGestureDetector mScaleDetector;
  private GvrView mGVRview;

  private float eyeZoom = 0.4f;
  private float eyeSide = 0.21f;
  private float eyeDown = 0.1f;
  private float zoom = 1.0f;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    //workaround for orientation bug
    setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

    mMoreButton = findViewById(R.id.more_button);
    mThumbnailButton = findViewById(R.id.thumbnail_button);
    mRecordButton = findViewById(R.id.record_button);
    mVRSetup = findViewById(R.id.vrLayout);
    mSurfaceView = findViewById(R.id.glsurfaceview);
    mSurfaceView.setOnTouchListener(this);
    mSurfaceView.setRenderer(this);

    //stereo view
    mGVRview = findViewById(R.id.gvr_view);
    mGVRview.setEGLConfigChooser(8, 8, 8, 8, 24, 8);
    mGVRview.setEGLContextClientVersion(3);
    mGVRview.setRenderer(new GvrView.Renderer() {
      @Override
      public void onDrawFrame(HeadTransform headTransform, Eye eye, Eye eye1) {
        float[] angles = new float[3];
        headTransform.getEulerAngles(angles, 0);
        depthmapRenderer.setAngles(angles);
      }

      @Override
      public void onFinishFrame(Viewport viewport) {
      }

      @Override
      public void onSurfaceChanged(int i, int i1) {
      }

      @Override
      public void onSurfaceCreated(EGLConfig eglConfig) {
      }

      @Override
      public void onRendererShutdown() {
      }
    });
    mGVRview.setTransitionViewEnabled(true);
    mGVRview.setDistortionCorrectionEnabled(true);
    if (mGVRview.setAsyncReprojectionEnabled(true))
      AndroidCompat.setSustainedPerformanceMode(this, true);
    setGvrView(mGVRview);

    getWindow().setNavigationBarColor(Color.BLACK);
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_TRANSLUCENT_STATUS);

    WindowManager.LayoutParams winParams = getWindow().getAttributes();
    winParams.rotationAnimation = WindowManager.LayoutParams.ROTATION_ANIMATION_CROSSFADE;
    getWindow().setAttributes(winParams);

    WindowManager.LayoutParams lp = getWindow().getAttributes();
    lp.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES;
    getWindow().setAttributes(lp);

    mScaleDetector = new ScaleGestureDetector(this, new ScaleGestureDetector.OnScaleGestureListener() {

      float last;

      @Override
      public void onScaleEnd(ScaleGestureDetector detector) {
      }
      @Override
      public boolean onScaleBegin(ScaleGestureDetector detector) {
        last = 0;
        return true;
      }
      @Override
      public boolean onScale(ScaleGestureDetector detector) {
        float f = detector.getScaleFactor() - 1.0f;
        zoom += f - last;
        if (zoom < 1) zoom = 1;
        if (zoom > 2) zoom = 2;
        last = f;
        return false;
      }
    });

    mMoreButton.setOnClickListener(v -> showMoreFeatures());

    mRecordButton.setOnClickListener(v -> {
      if (Recorder.isVideoRecording()) {
        mRecordButton.setVisibility(View.GONE);
        new Thread(() -> {
          Recorder.stopCapturingVideo(TofViewerActivity.this, true);
          runOnUiThread(() -> {
            mRecordButton.setBackgroundResource(R.drawable.ic_record);
            mMoreButton.setVisibility(View.VISIBLE);
            mRecordButton.setVisibility(View.VISIBLE);
            mThumbnailButton.setVisibility(View.VISIBLE);
          });
        }).start();
      } else {
        Recorder.setCustomRoot(getDataDir());
        Recorder.startCapturingVideo(TofViewerActivity.this, true);
        mRecordButton.setBackgroundResource(R.drawable.ic_stop);
        mMoreButton.setVisibility(View.GONE);
        mThumbnailButton.setVisibility(View.GONE);
      }
    });

    mThumbnailButton.setOnClickListener(viewPhoto -> {
      mMoreButton.setVisibility(View.GONE);
      mRecordButton.setVisibility(View.GONE);
      mThumbnailButton.setVisibility(View.GONE);
      new Thread(() -> {
        captureBitmap();
        runOnUiThread(() -> {
          mMoreButton.setVisibility(View.VISIBLE);
          mRecordButton.setVisibility(View.VISIBLE);
          mThumbnailButton.setVisibility(View.VISIBLE);
        });
      }).start();
    });

    if (recordNight) {
      lp.screenBrightness = 0;
      getWindow().setAttributes(lp);
      mRecordButton.setVisibility(View.GONE);
      mMoreButton.setVisibility(View.GONE);
      mThumbnailButton.setVisibility(View.GONE);
    }
    if (vrMode) {
      mRecordButton.setVisibility(View.GONE);
      mMoreButton.setVisibility(View.GONE);
      mThumbnailButton.setVisibility(View.GONE);
      mVRSetup.setVisibility(View.VISIBLE);
    }

    mSurfaceView.setOnLongClickListener(v -> {
      cancelRecordingNight();
      return false;
    });

    //VR calibration
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
    findViewById(R.id.down).setOnClickListener(v -> {
      eyeDown += 0.01f;
      SharedPreferences.Editor e = pref.edit();
      e.putFloat(KEY_EYE_DOWN, eyeDown);
      e.apply();
    });
    findViewById(R.id.up).setOnClickListener(v -> {
      eyeDown -= 0.01f;
      SharedPreferences.Editor e = pref.edit();
      e.putFloat(KEY_EYE_DOWN, eyeDown);
      e.apply();
    });
    findViewById(R.id.left).setOnClickListener(v -> {
      eyeSide -= 0.01f;
      if (eyeSide < 0.01f)
        eyeSide = 0.01f;
      SharedPreferences.Editor e = pref.edit();
      e.putFloat(KEY_EYE_SIDE, eyeSide);
      e.apply();
    });
    findViewById(R.id.right).setOnClickListener(v -> {
      eyeSide += 0.01f;
      SharedPreferences.Editor e = pref.edit();
      e.putFloat(KEY_EYE_SIDE, eyeSide);
      e.apply();
    });
    findViewById(R.id.reset).setOnClickListener(v -> {
      eyeZoom = 1.0f;
      eyeSide = 0.21f;
      eyeDown = 0.1f;
      SharedPreferences.Editor e = pref.edit();
      e.putFloat(KEY_EYE_ZOOM, eyeZoom);
      e.putFloat(KEY_EYE_DOWN, eyeDown);
      e.putFloat(KEY_EYE_SIDE, eyeSide);
      e.apply();
    });
    findViewById(R.id.zoomIn).setOnClickListener(v -> {
      eyeZoom += 0.01f;
      SharedPreferences.Editor e = pref.edit();
      e.putFloat(KEY_EYE_ZOOM, eyeZoom);
      e.apply();
    });
    findViewById(R.id.zoomOut).setOnClickListener(v -> {
      eyeZoom -= 0.01f;
      SharedPreferences.Editor e = pref.edit();
      e.putFloat(KEY_EYE_ZOOM, eyeZoom);
      e.apply();
    });
  }

  private void cancelRecordingNight() {
    if (recordNight) {
      new Thread(() -> {
        Recorder.stopCapturingVideo(this, true);
        try {
          Thread.sleep(500);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        System.exit(0);
      }).start();
    }
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus) {
    super.onWindowFocusChanged(hasFocus);
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
  }

  @Override
  public void onSurfaceCreated(GL10 gl, EGLConfig config) {
    // Prepare the rendering objects. This involves reading shaders, so may throw an IOException.
    try {
      depthmapRenderer.createOnGlThread(this);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void onSurfaceChanged(GL10 gl, int width, int height) {
    if (!connected) {
      updateParams();
      connected = true;
    }
  }

  @Override
  public void onDrawFrame(GL10 gl) {
    gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);

    if (vrMode) {
      renderEye(eyeZoom * 0.5f, -eyeSide, -eyeDown);
      renderEye(eyeZoom * 0.5f, eyeSide, -eyeDown);
    } else {

      Point size = new Point();
      size.x = mSurfaceView.getWidth();
      size.y = mSurfaceView.getHeight();
      int depthWidth = size.x;
      int depthHeight = size.x * 3 / 4;

      if (size.x > size.y) {
        float aspect = size.y / (float) size.x * (float) depthWidth / (float) depthHeight;
        int width = (int) (size.x * aspect * zoom);
        int height = (int) (size.y * zoom);
        GLES20.glEnable(GLES20.GL_SCISSOR_TEST);
        GLES20.glScissor((size.x - width) / 2, (size.y - height) / 2, width, height);
        GLES20.glViewport((size.x - width) / 2, (size.y - height) / 2, width, height);
      } else {
        float aspect = size.x / (float) size.y * (float) depthWidth / (float) depthHeight;
        int width = (int) (size.x * zoom);
        int height = (int) (size.y * aspect * zoom);
        GLES20.glEnable(GLES20.GL_SCISSOR_TEST);
        GLES20.glScissor((size.x - width) / 2, (size.y - height) / 2, width, height);
        GLES20.glViewport((size.x - width) / 2, (size.y - height) / 2, width, height);
      }
      depthmapRenderer.draw(this, zoom);
    }
    GLES20.glDisable(GLES20.GL_SCISSOR_TEST);

    if (mMakePhoto) {
      Recorder.capturePhoto(gl, mSurfaceView);
      mMakePhoto = false;
    } else if (recordNight) {
      Recorder.captureVideoFrame(gl, mSurfaceView, true, Recorder.getVideoFPS() * 60 / 5, true);
      gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
    } else {
      Recorder.captureVideoFrame(gl, mSurfaceView, false, 0, true);
    }
  }

  private void renderEye(float zoom, float dx, float dy) {
    Point size = new Point();
    size.x = mSurfaceView.getWidth();
    size.y = mSurfaceView.getHeight();
    int depthWidth = size.x;
    int depthHeight = size.x * 3 / 4;
    dx *= (float) size.x;
    dy *= (float) size.y;

    if (size.x > size.y) {
      float aspect = size.y / (float) size.x * (float) depthWidth / (float) depthHeight;
      int width = (int) (size.x * aspect * zoom);
      int height = (int) (size.y * zoom);
      GLES20.glEnable(GLES20.GL_SCISSOR_TEST);
      GLES20.glScissor((size.x - width) / 2 + (int) dx, (size.y - height) / 2 + (int) dy, width, height);
      GLES20.glViewport((size.x - width) / 2 + (int) dx, (size.y - height) / 2 + (int) dy, width, height);
    } else {
      float aspect = size.x / (float) size.y * (float) depthWidth / (float) depthHeight;
      int width = (int) (size.x * zoom);
      int height = (int) (size.y * aspect * zoom);
      GLES20.glEnable(GLES20.GL_SCISSOR_TEST);
      GLES20.glScissor((size.x - width) / 2 + (int) dx, (size.y - height) / 2 + (int) dy, width, height);
      GLES20.glViewport((size.x - width) / 2 + (int) dx, (size.y - height) / 2 + (int) dy, width, height);
    }
    depthmapRenderer.draw(this, 1);
  }

  @Override
  protected void onPause() {
    if (initialized && !isChangingConfigurations()) {
      if (recordNight) {
        cancelRecordingNight();
      } else {
        depthmapRenderer.closeCamera();
        System.exit(0);
      }
    }
    super.onPause();
  }

  @Override
  protected void onResume() {
    super.onResume();
    for (String permission : permissions) {
      if (checkSelfPermission(permission) != PackageManager.PERMISSION_GRANTED) {
        requestPermissions(permissions, REQUEST_PERMISSIONS);
        return;
      }
    }

    if (!initialized) {
      mSurfaceView.setVisibility(View.VISIBLE);
      mGVRview.setVisibility(View.VISIBLE);

      depthmapRenderer.initCamera(this);
      depthmapRenderer.openCamera(this);
      initialized = true;
    }
  }

  @Override
  public void onBackPressed() {
    super.onBackPressed();
    if (recordNight) {
      cancelRecordingNight();
    } else {
      depthmapRenderer.closeCamera();
      System.exit(0);
    }
  }

  private void captureBitmap() {
    synchronized (this) {
      mMakePhoto = true;
    }
    try {
      while (mMakePhoto) {
        Thread.sleep(20);
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  private void showMoreFeatures() {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);

    CharSequence[] items = {
            getString(R.string.scheme),
            getString(R.string.sleep),
            getString(R.string.vr)
    };

    AlertDialog.Builder dialog = new AlertDialog.Builder(TofViewerActivity.this);
    dialog.setTitle(R.string.app_name);
    dialog.setItems(items, (dialog1, which) -> {
      AlertDialog.Builder dlg = new AlertDialog.Builder(TofViewerActivity.this);
      switch (which) {
        case 0:
          dlg.setTitle(R.string.scheme);
          dlg.setItems(R.array.scheme_variants, (dialogInterface, i) -> {
            SharedPreferences.Editor e = PreferenceManager.getDefaultSharedPreferences(TofViewerActivity.this).edit();
            e.putInt(KEY_SCHEME, i);
            e.commit();
            updateParams();
          });
          dlg.setNegativeButton(android.R.string.cancel, null);
          dlg.show();
          break;
        case 1:
          if (Settings.System.getInt(getContentResolver(), Settings.Global.AIRPLANE_MODE_ON, 0) == 0) {
            dlg.setTitle(R.string.sleep);
            dlg.setMessage(R.string.sleep_airplane);
            dlg.setPositiveButton(android.R.string.ok, null);
            dlg.show();
            return;
          }

          dlg.setTitle(R.string.sleep);
          dlg.setMessage(R.string.sleep_description);
          dlg.setNegativeButton(android.R.string.cancel, null);
          dlg.setPositiveButton(android.R.string.ok, (dialogInterface, i) -> {
            recordNight = true;
            //setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR_LANDSCAPE);
            new Thread(() -> {
              try {
                Thread.sleep(500);
              } catch (InterruptedException e) {
                e.printStackTrace();
              }
              runOnUiThread(() -> {
                WindowManager.LayoutParams lp = getWindow().getAttributes();
                lp.screenBrightness = 0;
                getWindow().setAttributes(lp);
                Recorder.startCapturingVideo(TofViewerActivity.this, false);
                mRecordButton.setVisibility(View.GONE);
                mMoreButton.setVisibility(View.GONE);
                mThumbnailButton.setVisibility(View.GONE);
              });
            }).start();
          });
          dlg.show();
          break;
        case 2:
          dlg.setTitle(R.string.vr);
          dlg.setMessage(R.string.vr_description);
          dlg.setNegativeButton(android.R.string.cancel, null);
          dlg.setPositiveButton(android.R.string.ok, (dialogInterface, i) -> {
            vrMode = true;
            //setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR_LANDSCAPE);
            new Thread(() -> {
              try {
                Thread.sleep(500);
              } catch (InterruptedException e) {
                e.printStackTrace();
              }
              runOnUiThread(() -> {
                eyeDown = pref.getFloat(KEY_EYE_DOWN, eyeDown);
                eyeSide = pref.getFloat(KEY_EYE_SIDE, eyeSide);
                eyeZoom = pref.getFloat(KEY_EYE_ZOOM, eyeZoom);
                mRecordButton.setVisibility(View.GONE);
                mMoreButton.setVisibility(View.GONE);
                mThumbnailButton.setVisibility(View.GONE);
                mVRSetup.setVisibility(View.VISIBLE);
              });
            }).start();
          });
          dlg.show();
          break;
      }
    });
    dialog.show();
  }

  private void updateParams() {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
    int scheme = pref.getInt(KEY_SCHEME, 0);
    depthmapRenderer.setColorScheme(scheme);
  }

  @Override
  public boolean onTouch(View v, MotionEvent event) {
    mScaleDetector.onTouchEvent(event);
    return false;
  }
}
