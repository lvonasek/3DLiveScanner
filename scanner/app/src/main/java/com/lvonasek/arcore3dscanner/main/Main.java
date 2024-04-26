package com.lvonasek.arcore3dscanner.main;

import android.app.ActivityManager;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.location.Location;
import android.os.Build;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Display;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.RelativeLayout;
import android.widget.SeekBar;
import android.widget.TextView;

import androidx.core.content.FileProvider;


import com.lvonasek.arcore3dscanner.R;
import com.lvonasek.arcore3dscanner.sketchfab.OAuth;
import com.lvonasek.arcore3dscanner.ui.AbstractActivity;
import com.lvonasek.arcore3dscanner.ui.CommonDialogs;
import com.lvonasek.arcore3dscanner.ui.Service;
import com.lvonasek.gles.GLESSurfaceView;
import com.lvonasek.record.Recorder;
import com.lvonasek.utils.Compass;
import com.lvonasek.utils.Compatibility;
import com.lvonasek.utils.GPS;
import com.lvonasek.arcore3dscanner.BuildConfig;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class Main extends AbstractActivity implements View.OnClickListener,
        GLESSurfaceView.Renderer, View.OnTouchListener {

  private DistanceMeasuring mDistance;
  private GPS mGPS;
  private ProgressBar mProgress;
  private GLESSurfaceView mGLView;
  private String mToLoad, mToPostprocess, mOpenedFile;
  private boolean m3drRunning = false;

  private FrameLayout mHandMotionView;
  private LinearLayout mLayoutQuickMenu;
  private LinearLayout mLayoutRec;
  private LinearLayout mLayoutUndo;
  private LinearLayout mLayoutView;
  private LinearLayout mLayoutWait;
  private ImageButton mViewButton;
  private ImageButton mToggleButton;
  private ImageButton mUndoButton;
  private Button mEditorButton;
  private Button mThumbnailButton;
  private float mRes = 0.02f;
  private int mViewCamera = 0;

  private RelativeLayout mLayoutEditor;
  private ArrayList<Button> mEditorAction;
  private CheckBox mEditorDeselect;
  private TextView mEditorMsg;
  private SeekBar mEditorSeek;
  private Editor mEditor;
  private Indicators mIndicators;
  private CameraControl mCameraControl;

  // AR Service connection.
  boolean mInitialised = false;
  boolean mARBinded = false;
  boolean mIgnoreSaving = false;
  boolean mInitialisedRecBar = false;
  boolean mRestoreViewOnResume = false;
  long mLastClick = 0;
  float mLastClickX = 0;
  float mLastClickY = 0;
  boolean mAnchors = false;
  boolean mLongClick = false;
  boolean mSelection = false;
  boolean mShowGrid = false;
  boolean mPhotoMode = false;
  boolean mRecording = false;
  float mCameraRecordYaw = 0;

  int getARMode() {
    int mode = getBackend(this) * 3; //0 = GOOGLE_SFM, 3 = HUAWEI_SFM
    if (isFaceModeOn(this)) {
      mCameraControl.updateView(CameraControl.ViewMode.FACE);
      if (Compatibility.shouldUseHuawei(this))
        mode = 5; //HUAWEI_FACE
      else
        mode = 2; //GOOGLE_FACE
    }
    else if (isTofOn(this))
      mode++; //1 = GOOGLE_TOF, 4 = HUAWEI_TOF

    //HUAWEI_SFM
    if (mode == 3) {
      if (Compatibility.isARCoreSupportedAndUpToDate(this)) {
        mode = 0; //GOOGLE_SFM
      }
    }
    //GOOGLE_SFM
    if (mode == 0) {
      if (!Compatibility.isGoogleDepthSupported(this)) {
        mRes *= 1.5f;
      }
    }
    return mode;
  }

  void bindAR() {
    int mode = getARMode();
    if (!isFaceModeOn(this)) {
      runOnUiThread(() -> mViewButton.setVisibility(View.VISIBLE));
    }

    boolean texturize = (mToPostprocess != null) || (Math.abs(Service.getRunning(this)) == Service.SERVICE_SAVE);
    double res = mRes, dmin = 0.01f, dmax = 7;
    mCameraControl.setOffset(0);
    if (mRes > 0.0099f) {
      mCameraControl.setOffset(mRes * 100);
    }
    //change resolution for HUAWEI_SFM
    if (mode == 3) { res *= 2.0f; }

    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
    mAnchors      = pref.getBoolean(getString(R.string.pref_anchor), false);
    int noise     = Integer.parseInt(pref.getString(getString(R.string.pref_noise), "9"));
    boolean analy = pref.getBoolean(getString(R.string.pref_subset), false);
    boolean clear = pref.getBoolean(getString(R.string.pref_clear), true);
    boolean disto = false;
    boolean poses = pref.getBoolean(getString(R.string.pref_slow), false);
    boolean offst = pref.getBoolean(getString(R.string.pref_offset), true) && !poses;
    boolean holes = mode == 3; // HUAWEI_SFM
    boolean flash = pref.getBoolean(getString(R.string.pref_flash), false) && !isFaceModeOn(this) && !texturize;
    boolean poiss = pref.getBoolean(getString(R.string.pref_poisson), false);
    dmax = Integer.parseInt(pref.getString(getString(R.string.pref_limit), "4"));

    if (pref.getBoolean(getString(R.string.pref_gps), false)) {
      mGPS = new GPS();
      mGPS.start(this, 100);
    }

    if (texturize) {
      m3drRunning = false;
      mIgnoreSaving = true;
    } else if (!isFaceModeOn(this)) {
      m3drRunning = true;
      deleteRecursive(getTempPath());
      getTempPath().mkdirs();
    }

    int decimation = Integer.parseInt(pref.getString(getString(R.string.pref_decimation), "1"));
    ActivityManager actManager = (ActivityManager) getSystemService(Context.ACTIVITY_SERVICE);
    ActivityManager.MemoryInfo memInfo = new ActivityManager.MemoryInfo();
    actManager.getMemoryInfo(memInfo);
    int MB = (int) (memInfo.totalMem / 1048576L);
    int texture_max = Math.min(Math.max(1, MB / 512), 8);
    switch (pref.getString(getString(R.string.pref_textures), "4")) {
      case "1":
        texture_max = 1;
        break;
      case "4":
        texture_max = 4;
        break;
    }
    int texture_res = 2048;

    String t = mToPostprocess != null ? mToPostprocess : getTempPath().getAbsolutePath();
    JNI.motionTrackingMessages = !isFaceModeOn(this);
    JNI.setTextureParams(decimation, texture_res, texture_max);
    if (!JNI.onARServiceConnected(this, res, dmin, dmax, noise, holes, poses, disto, offst,
                              flash, mode, clear, t.getBytes())) {
      showAndroidBugDialog();
      return;
    }
    JNI.onToggleButtonClicked(m3drRunning);
    mCameraControl.updateView();
    final File input = new File(Service.getLink(Main.this));
    final File obj = new File(getTempPath(), System.currentTimeMillis() + Exporter.EXT_OBJ);

    if (m3drRunning) {
      runOnUiThread(() -> mHandMotionView.setVisibility(View.VISIBLE));
    }

    if (texturize) {
      String export = pref.getString(getString(com.lvonasek.arcore3dscanner.R.string.pref_mode), "realtime");
      Service.process(getString(R.string.postprocessing), Service.SERVICE_POSTPROCESS,
              Main.this, () -> {
                mGLView.stop();
                finish();

                JNI.motionTrackingMessages = false;
                if (export.compareTo("exp_floorplan") == 0) {
                  String path = mToPostprocess + "/";
                  JNI.extract(path.getBytes(), Exporter.EXPORT_TYPE_FLOORPLAN);
                  Service.finish(path + "floorplan.obj");
                } else if (export.compareTo("exp_pointcloud") == 0) {
                  if (mToPostprocess != null) {
                    JNI.onUndoButtonClicked(false, true);
                  }
                  String path = mToPostprocess + "/";
                  JNI.extract((path + "pointcloud.ply").getBytes(), Exporter.EXPORT_TYPE_POINTCLOUD);
                  Service.finish(path + "pointcloud.ply");
                } else {
                  if (mToPostprocess != null) {
                    JNI.onUndoButtonClicked(false, false);
                  }

                  File i = input;
                  File o = obj;
                  if (mToPostprocess != null) {
                    i = new File(mToPostprocess, "model" + Exporter.EXT_OBJ);
                    o = new File(mToPostprocess, System.currentTimeMillis() + Exporter.EXT_OBJ);
                    if (!JNI.save(i.getAbsolutePath().getBytes())) {
                      Service.reset(Main.this);
                      System.exit(0);
                    }
                  }
                  JNI.texturize(i.getAbsolutePath().getBytes(), o.getAbsolutePath().getBytes(), poiss, analy);
                  Service.finish(o.getAbsolutePath());
                }
              });
    } else {
      Main.this.runOnUiThread(() -> mProgress.setVisibility(View.GONE));
    }
    mInitialised = true;
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    // Setup UI elements and listeners.
    mHandMotionView = findViewById(R.id.ar_hand_layout);
    mLayoutQuickMenu = findViewById(R.id.layout_quickmenu);
    mLayoutRec = findViewById(R.id.layout_rec);
    mLayoutUndo = findViewById(R.id.layout_undo);
    mLayoutView = findViewById(R.id.layout_view);
    mLayoutWait = findViewById(R.id.layout_wait);
    findViewById(R.id.clear_button).setOnClickListener(this);
    findViewById(R.id.save_button).setOnClickListener(this);
    mViewButton = findViewById(R.id.view_button);
    mViewButton.setVisibility(View.GONE);
    mViewButton.setOnClickListener(this);
    mToggleButton = findViewById(R.id.toggle_button);
    mToggleButton.setOnClickListener(this);
    mUndoButton = findViewById(R.id.undo_button);
    mUndoButton.setOnClickListener(this);
    findViewById(R.id.undo_apply).setOnClickListener(this);
    findViewById(R.id.undo_cancel).setOnClickListener(this);
    findViewById(R.id.undo_back).setOnClickListener(this);
    findViewById(R.id.undo_back_fast).setOnClickListener(this);
    findViewById(R.id.undo_fwd).setOnClickListener(this);
    findViewById(R.id.undo_fwd_fast).setOnClickListener(this);

    if (isFaceModeOn(this)) {
      findViewById(R.id.clear_button).setVisibility(View.GONE);
      findViewById(R.id.view_button).setVisibility(View.GONE);
      mToggleButton.setVisibility(View.GONE);
      mUndoButton.setVisibility(View.GONE);
    }

    CheckBox photoMode = findViewById(R.id.photo_mode);
    photoMode.setOnCheckedChangeListener((buttonView, isChecked) -> {
      mPhotoMode = isChecked;
      m3drRunning = false;
      mToggleButton.setImageResource(isChecked ? R.drawable.ic_capture : R.drawable.ic_record);
      JNI.setPhotoMode(isChecked);
    });
    CheckBox showNormals = findViewById(R.id.show_normals);
    showNormals.setOnCheckedChangeListener((buttonView, isChecked) -> mEditor.swapNormals());

    // Buttons and record info
    mEditorButton = findViewById(R.id.editor_button);
    mThumbnailButton = findViewById(R.id.thumbnail_button);

    // Editor
    mLayoutEditor = findViewById(R.id.layout_editor);
    mEditorDeselect = findViewById(R.id.editorDeselect);
    mEditorMsg = findViewById(R.id.editorMsg);
    mEditorSeek = findViewById(R.id.editorSeek);
    mEditorAction = new ArrayList<>();
    mEditorAction.add(findViewById(R.id.editor0));
    mEditorAction.add(findViewById(R.id.editor1));
    mEditorAction.add(findViewById(R.id.editor2));
    mEditorAction.add(findViewById(R.id.editor3));
    mEditorAction.add(findViewById(R.id.editor4));
    mEditorAction.add(findViewById(R.id.editor1a));
    mEditorAction.add(findViewById(R.id.editor1b));
    mEditorAction.add(findViewById(R.id.editor1c));
    mEditorAction.add(findViewById(R.id.editor1d));
    mEditorAction.add(findViewById(R.id.editor1e));
    mEditorAction.add(findViewById(R.id.editor1f));
    mEditorAction.add(findViewById(R.id.editor2a));
    mEditorAction.add(findViewById(R.id.editor2b));
    mEditorAction.add(findViewById(R.id.editor2c));
    mEditorAction.add(findViewById(R.id.editor2d));
    mEditorAction.add(findViewById(R.id.editor2e));
    mEditorAction.add(findViewById(R.id.editor3a));
    mEditorAction.add(findViewById(R.id.editor3b));
    mEditorAction.add(findViewById(R.id.editor3c));
    mEditorAction.add(findViewById(R.id.editor4a));
    mEditorAction.add(findViewById(R.id.editor4b));
    mEditorAction.add(findViewById(R.id.editor4c));
    mEditorAction.add(findViewById(R.id.editor4d));
    mEditorAction.add(findViewById(R.id.editorX));
    mEditorAction.add(findViewById(R.id.editorY));
    mEditorAction.add(findViewById(R.id.editorZ));

    // OpenGL view where all of the graphics are drawn
    if (!isCameraFeedOn(this)) {
      mViewCamera = 3;
    }
    mGLView = findViewById(R.id.gl_surface_view);
    mGLView.setRenderer(this);
    mGLView.setOnTouchListener(this);

    mDistance = findViewById(R.id.distance);
    mEditor = findViewById(R.id.editor);
    mProgress = findViewById(R.id.progressBar);
    mCameraControl = new CameraControl(this, mDistance, mEditor);

    //open file
    mToLoad = null;
    mToPostprocess = null;
    String filename = getIntent().getStringExtra(FILE_KEY);
    if ((filename != null) && (filename.length() > 0))
    {
      m3drRunning = false;
      try
      {
        if (filename.endsWith(Exporter.EXT_DATASET)) {
          mLayoutView.setVisibility(View.GONE);
          mLayoutRec.setVisibility(View.GONE);
          mLayoutWait.setVisibility(View.VISIBLE);
          mRes = getResolution(this);
          mProgress.setVisibility(View.VISIBLE);
          mToPostprocess = filename;
          return;
        }
        else if (new File(filename).isFile())
          mToLoad = new File(filename).toString();
        else
          mToLoad = getModel(new File(filename)).toString();
        setViewerMode(mToLoad);
      } catch (Exception e) {
        Log.e(TAG, "Unable to load " + filename);
        e.printStackTrace();
      }
    }
    else {
      // Workaround to incorrect uv transition
      CommonDialogs.setImmersive(getWindow());

      mLayoutView.setVisibility(View.GONE);
      mRes = getResolution(this);
    }
    mProgress.setVisibility(View.VISIBLE);
  }

  @Override
  public void onClick(View v) {
    int id = v.getId();

    if (id == R.id.toggle_button) {
      if (mPhotoMode) {
        JNI.onToggleButtonClicked(true);
      } else {
        m3drRunning = !m3drRunning;
        JNI.onToggleButtonClicked(m3drRunning);
      }
      runOnUiThread(() -> mHandMotionView.setVisibility(View.GONE));
    } else if (id == R.id.clear_button) {
      pauseScanning();
      if (JNI.getScanSize() > 0) {
        CommonDialogs.confirmDialog(this, R.string.scan_discard, JNI::onClearButtonClicked);
      }
    } else if (id == R.id.view_button) {
      boolean visible = mLayoutQuickMenu.getVisibility() == View.VISIBLE;
      mLayoutQuickMenu.setVisibility(visible ? View.GONE : View.VISIBLE);
      mViewButton.setImageResource(visible ? R.drawable.ic_arrow_up : R.drawable.ic_arrow_down);
    } else if (id == R.id.undo_button) {
      pauseScanning();
      mIndicators.setOverrideMessage(getString(R.string.scan_rewind));
      mLayoutRec.setVisibility(View.GONE);
      mLayoutUndo.setVisibility(View.VISIBLE);
      mLayoutQuickMenu.setVisibility(View.GONE);
      mViewButton.setImageResource(R.drawable.ic_arrow_up);
    } else if (id == R.id.undo_apply) {
      mIndicators.setOverrideMessage(null);
      mLayoutUndo.setVisibility(View.GONE);
      mLayoutWait.setVisibility(View.VISIBLE);
      new Thread(() -> {
        JNI.onUndoButtonClicked(true, true);
        runOnUiThread(() -> {
          mLayoutRec.setVisibility(View.VISIBLE);
          mLayoutWait.setVisibility(View.GONE);
        });
      }).start();
    } else if (id == R.id.undo_cancel) {
      mIndicators.setOverrideMessage(null);
      new Thread(() -> {
        JNI.onUndoPreviewUpdate(Integer.MAX_VALUE);
        runOnUiThread(() -> {
          mLayoutRec.setVisibility(View.VISIBLE);
          mLayoutUndo.setVisibility(View.GONE);
        });
      }).start();
    } else if (id == R.id.undo_back) {
      new Thread(() -> JNI.onUndoPreviewUpdate(-1)).start();
    } else if (id == R.id.undo_back_fast) {
      new Thread(() -> JNI.onUndoPreviewUpdate(-10)).start();
    } else if (id == R.id.undo_fwd) {
      new Thread(() -> JNI.onUndoPreviewUpdate(1)).start();
    } else if (id == R.id.undo_fwd_fast) {
      new Thread(() -> JNI.onUndoPreviewUpdate(10)).start();
    } else if (id == R.id.save_button) {
      if (isFaceModeOn(this)) {
        save();
        finish();
      } else {
        pauseScanning();
        CommonDialogs.confirmDialog(this, R.string.scan_finish, new Runnable() {
          @Override
          public void run() {
            save();
            finish();
          }
        });
      }
    }

    if (!mPhotoMode) {
      mToggleButton.setImageResource(m3drRunning ? R.drawable.ic_pause : R.drawable.ic_record);
    }
  }


  @Override
  public boolean onKeyUp(int keyCode, KeyEvent keyEvent) {
    if (keyCode == KeyEvent.KEYCODE_ENTER) {
      onClick(mToggleButton);
      mToggleButton.requestFocus();
      return true;
    }
    return super.onKeyUp(keyCode, keyEvent);
  }

  @Override
  public int getNavigationBarColor() {
    return getStatusBarColor();
  }

  @Override
  public int getStatusBarColor() {
    return Color.BLACK;
  }

  public GLESSurfaceView getGLView() {
    return mGLView;
  }

  @Override
  protected void onResume() {
    super.onResume();

    if (mRestoreViewOnResume)
      mCameraControl.restoreView();

    if (mCameraControl.isViewMode()) {
      if (mToLoad != null) {
        final String file = "" + mToLoad;
        mOpenedFile = mToLoad;
        mToLoad = null;

        new Thread(() -> {
          if (!JNI.load(file.getBytes())) {
            showAndroidBugDialog();
          };
          if (mOpenedFile.endsWith(Exporter.EXT_OBJ))
            mCameraControl.captureBitmap(false, mOpenedFile);
          Main.this.runOnUiThread(() -> mProgress.setVisibility(View.GONE));
          mInitialised = true;
        }).start();
      }
      setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_UNSPECIFIED);
    } else {
      CommonDialogs.setImmersive(getWindow());
      mIndicators = new Indicators(this);
    }
  }

  @Override
  public void onBackPressed() {
    if (mARBinded && !isFaceModeOn(this)) {
      pauseScanning();
      if (JNI.getScanSize() > 0) {
        CommonDialogs.confirmDialog(this, R.string.scan_discard, new Runnable() {
          @Override
          public void run() {
            System.exit(0);
          }
        });
      } else {
        System.exit(0);
      }
    } else {
      super.onBackPressed();
    }
  }

  private void pauseScanning() {
    if (!mPhotoMode) {
      m3drRunning = false;
      JNI.onToggleButtonClicked(m3drRunning);
      runOnUiThread(() -> {
        mToggleButton.setImageResource(m3drRunning ? R.drawable.ic_pause : R.drawable.ic_record);
      });
    }
  }

  @Override
  protected void onPause() {
    super.onPause();

    //stop GPS
    if (mGPS != null) {
      mGPS.stop();
    }

    //do not do anything if returned from VR
    if (mIgnoreSaving) {
      mIgnoreSaving = false;
    }

    //pause scanning
    else if (mARBinded && !isFaceModeOn(this)) {
      m3drRunning = false;
      JNI.onToggleButtonClicked(m3drRunning);
      mToggleButton.setImageResource(m3drRunning ? R.drawable.ic_pause : R.drawable.ic_record);
      JNI.onPause();
    } else {
      System.exit(0);
    }
  }

  private Runnable onLongClick = () -> {
    long timestamp = System.currentTimeMillis();
    while (System.currentTimeMillis() - timestamp < 500) {
      try {
        Thread.sleep(10);
      } catch (Exception e) {
        e.printStackTrace();
      }
      if (!mLongClick) {
        return;
      }
    }
    //Place for long click actions
    mSelection = true;
  };

  @Override
  public boolean onTouch(View v, MotionEvent event) {
    if (mRecording) {
      return true;
    } else if (mEditor.initialized())
    {
      mEditor.touchEvent(event);
      if (mEditor.movingLocked())
        return true;
    }

    //double click to zoom
    Display d = getWindowManager().getDefaultDisplay();
    float move = 0;
    move += Math.abs(mLastClickX - event.getX()) / (float)d.getWidth();
    move += Math.abs(mLastClickY - event.getY()) / (float)d.getHeight();
    if (event.getAction() == MotionEvent.ACTION_DOWN) {
      if (System.currentTimeMillis() - mLastClick < 500) {
        mCameraControl.onDoubleClick(move);
      }
      //begin long click
      else if (!mEditor.initialized()) {
        mLongClick = true;
        new Thread(onLongClick).start();
      }
      mLastClick = System.currentTimeMillis();
      mLastClickX = event.getX();
      mLastClickY = event.getY();

    } else if (event.getAction() == MotionEvent.ACTION_UP) {
      if (move > 0.05f) {
        mLastClick = 0;
      }
      //short click
      else if (System.currentTimeMillis() - mLastClick < 500) {
        if (mSelection) {
          new Thread(() -> {
            JNI.completeSelection(true);
            mSelection = false;
          }).start();
        } else if (!mCameraControl.isViewMode()) {
            if (isCameraFeedOn(this)) {
              if (mViewCamera == 0) {
                if (mLastClickY < mGLView.getHeight() / 6) {
                  if (mLastClickX < mGLView.getWidth() / 6) {
                    mViewCamera = 1;
                  } else if (mLastClickX > 5 * mGLView.getWidth() / 6) {
                    mViewCamera = 2;
                  }
                }
              } else {
                mViewCamera = 0;
              }
            }
        }
      }
      mLongClick = false;
    }
    //cancel long click
    else if (event.getAction() == MotionEvent.ACTION_MOVE) {
      if (move > 0.05f) {
        mLongClick = false;
      }
    }

    //camera control gestures
    mCameraControl.updateMotion(event);
    return true;
  }

  private void setViewerMode(final String filename)
  {
    boolean floorplan = new File(new File(filename).getParentFile(), "floorplan.png").exists();
    boolean face = new File(new File(filename).getParentFile(), "model_face.jpg").exists();
    boolean pointcloud = filename.endsWith(Exporter.EXT_PLY);
    if (mIndicators != null) {
      mIndicators.disable();
    }
    mLayoutRec.setVisibility(View.GONE);
    if (face || floorplan)
      mLayoutView.setVisibility(View.GONE);

    if (!face && !floorplan && !pointcloud)
      mEditorButton.setVisibility(View.VISIBLE);
    mEditorButton.setOnClickListener(view -> {
      Display display = getWindowManager().getDefaultDisplay();
      ViewGroup.LayoutParams lp = mLayoutView.getLayoutParams();
      lp.width = Math.min(display.getWidth(), display.getHeight()) * 2 / 3;
      mLayoutView.setLayoutParams(lp);

      mDistance.reset();
      mCameraControl.getVRButton().setVisibility(View.GONE);
      mThumbnailButton.setVisibility(View.GONE);
      mEditorButton.setVisibility(View.GONE);
      mLayoutEditor.setVisibility(View.VISIBLE);
      getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
      setOrientation(false, Main.this);
      mEditor.init(mEditorAction, mEditorMsg, mEditorSeek, mProgress, mEditorDeselect,Main.this);
    });
    if (!face && !floorplan && !pointcloud && Compatibility.isPlayStoreSupported(this))
      mCameraControl.getVRButton().setVisibility(View.VISIBLE);
    mCameraControl.getVRButton().setOnClickListener(view -> {
      mDistance.reset();
      mIgnoreSaving = true;
      mRestoreViewOnResume = true;
      mCameraControl.enterVR(filename);
    });

    if (!pointcloud) {
      mThumbnailButton.setVisibility(View.VISIBLE);
      mThumbnailButton.setOnClickListener(view -> {
        mDistance.reset();
        CharSequence[] items;
        if (isProVersion(this)) {
          items = new CharSequence[]{
                  getString(R.string.sketchfab_dialog_title),
                  getString(R.string.screenshot),
                  getString(R.string.videoshot)
          };
        } else {
          items = new CharSequence[]{
                  getString(R.string.sketchfab_dialog_title),
                  getString(R.string.screenshot)
          };
        }

        AlertDialog.Builder dialog = new AlertDialog.Builder(this);
        dialog.setTitle(R.string.share_via);
        dialog.setItems(items, (dialog1, which) -> {
          switch (which) {
            case 0:
              mLayoutView.setVisibility(View.GONE);
              mProgress.setVisibility(View.VISIBLE);
              mEditorButton.setVisibility(View.GONE);
              mThumbnailButton.setVisibility(View.GONE);
              mIgnoreSaving = true;
              new Thread(() -> {
                File folder = new File(mOpenedFile).getParentFile();
                if ((folder == null) || (folder.getAbsolutePath().length() <= getPath(false).length())) {
                  folder = new File(getPath(false));
                }
                final String zip = Exporter.compressModel(folder);
                runOnUiThread(() -> {
                  Intent intent = new Intent(Main.this, OAuth.class);
                  intent.putExtra(AbstractActivity.FILE_KEY, zip);
                  startActivity(intent);
                  finish();
                });
              }).start();
              break;
            case 1:
              captureScreenshot();
              break;
            case 2:
              mCameraRecordYaw = 0;
              mLayoutView.setVisibility(View.GONE);
              mProgress.setVisibility(View.VISIBLE);
              mEditorButton.setVisibility(View.GONE);
              mThumbnailButton.setVisibility(View.GONE);
              mRecording = true;
              Recorder.setVideoDownscale(1280);
              Recorder.setVideoFPS(30);
              Recorder.startCapturingVideo(Main.this, false);
              break;
          }
        });
        dialog.show();
      });
    }
    mCameraControl.setViewerMode(face, floorplan);
  }

  private void captureScreenshot() {
    mProgress.setVisibility(View.VISIBLE);
    mThumbnailButton.setVisibility(View.GONE);
    new Thread(() -> {
      mIgnoreSaving = true;
      mCameraControl.captureBitmap(true, mOpenedFile);
      runOnUiThread(() -> {
        mProgress.setVisibility(View.GONE);
        mThumbnailButton.setVisibility(View.VISIBLE);
      });
    }).start();
  }

  @Override
  public synchronized void onDrawFrame(GL10 gl) {
    boolean texturize = (mToPostprocess != null) || (Math.abs(Service.getRunning(this)) == Service.SERVICE_SAVE);
    if (texturize) {
      return;
    }

    boolean face = mCameraControl.getViewMode() == CameraControl.ViewMode.FACE;
    boolean grid = mShowGrid && !face && !mRecording;
    if (JNI.onGlSurfaceDrawFrame(face, Compass.getValue(), mViewCamera, mAnchors, grid, !mRecording)) {
      runOnUiThread(() -> mHandMotionView.setVisibility(View.GONE));
    }
    if (JNI.didARjump()) {
      m3drRunning = false;
      JNI.onToggleButtonClicked(false);
      runOnUiThread(() -> mToggleButton.setImageResource(m3drRunning ? R.drawable.ic_pause : R.drawable.ic_record));
    }

    if (mRecording) {
      if (mCameraRecordYaw > 360) {
        mCameraRecordYaw = Integer.MAX_VALUE;
        mCameraControl.updateViewYaw(0);
        mRecording = false;
        new Thread(() -> {
          Recorder.stopCapturingVideo(Main.this, false);
          runOnUiThread(() -> {
            mLayoutView.setVisibility(View.VISIBLE);
            mProgress.setVisibility(View.GONE);
            mEditorButton.setVisibility(View.VISIBLE);
            mThumbnailButton.setVisibility(View.VISIBLE);
            mIgnoreSaving = true;

            File file = Recorder.getVideoFile();
            if (file != null) {
              Intent intent = new Intent(Intent.ACTION_SEND);
              intent.setType("video/mp4");
              intent.putExtra(Intent.EXTRA_STREAM, FileProvider.getUriForFile(Main.this, BuildConfig.APPLICATION_ID + ".provider", file));
              startActivity(Intent.createChooser(intent, getString(com.lvonasek.arcore3dscanner.R.string.share_via)));
            }
          });
        }).start();
      } else if (mCameraRecordYaw < Integer.MAX_VALUE * 0.5f) {
        Recorder.captureVideoFrame(gl, mGLView, false, 1, false);
        mCameraRecordYaw += 1;
        mCameraControl.updateViewYaw(mCameraRecordYaw);
      }
    } else {
      mCameraControl.updateCapture(gl, mGLView);
      mCameraControl.updateButtons();
    }
  }

  @Override
  public void onSurfaceCreated(GL10 gl10, EGLConfig eglConfig) {
    if(!mCameraControl.isViewMode() && !mInitialised && !mARBinded) {
      bindAR();
      mARBinded = true;
    }
  }

  @Override
  public void onSurfaceChanged(GL10 gl, int width, int height) {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
    boolean poses = pref.getBoolean(getString(R.string.pref_slow), false);
    boolean fullhd = pref.getBoolean(getString(R.string.pref_fullhd), false) || isFaceModeOn(this) || poses;
    mShowGrid = pref.getBoolean(getString(R.string.pref_grid), true);
    JNI.onGlSurfaceChanged(width, height, fullhd);

    //setup record bar
    if (!mInitialisedRecBar) {
      mInitialisedRecBar = true;
      DisplayMetrics displayMetrics = new DisplayMetrics();
      getWindowManager().getDefaultDisplay().getMetrics(displayMetrics);
      if (height != displayMetrics.heightPixels) {
        runOnUiThread(() -> {
          mLayoutRec.setMinimumHeight((int) (getNavigationBarHeight() + convertDpToPx(45 + 4)));
          mLayoutUndo.setMinimumHeight((int) (getNavigationBarHeight() + convertDpToPx(45 + 4)));
        });
      }
    }
  }

  private void save()
  {
    //save GPS coordinate
    if (mGPS != null) {
      mGPS.stop();
      Location gps = mGPS.getLastLocation();
      try {
        FileOutputStream info = new FileOutputStream(new File(getTempPath(), "position.txt").getAbsolutePath());
        String lat = Double.toString(gps.getLatitude()).replace(',', '.');
        String lon = Double.toString(gps.getLongitude()).replace(',', '.');
        info.write((lon + " " + lat).getBytes());
        info.close();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    mViewCamera = -1;

    //save face scan
    if (isFaceModeOn(this)) {
      JNI.onToggleButtonClicked(false);
      File input = new File(getTempPath(), "model" + Exporter.EXT_OBJ);
      if (JNI.save(input.getAbsolutePath().getBytes()))
        Service.forceState(this,getTempPath().getAbsolutePath() + "/" + input.getName(), Service.SERVICE_POSTPROCESS);
      System.exit(0);
    }
    //save dataset
    else if (isPostProcessLaterOn(this)) {
      Service.process(getString(R.string.saving), Service.SERVICE_SAVE, this, () -> {
        JNI.onToggleButtonClicked(false);
        mGLView.stop();
        finish();
        Date date = new Date() ;
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
        final String filename = dateFormat.format(date);
        File input = new File(getTempPath(), "model" + Exporter.EXT_OBJ);
        if (JNI.save(input.getAbsolutePath().getBytes())) {
          File dataset = new File(getPath(false), filename + Exporter.EXT_DATASET);
          if (getTempPath().renameTo(dataset)) {
            Log.d(TAG, "Datased saved");
            for (File file : dataset.listFiles()) {
              if (file.getAbsolutePath().endsWith(".bin")) {
                if (file.delete()) {
                  Log.d(TAG, file + " deleted");
                }
              }
            }
          }
        }
        Service.reset(Main.this);
        System.exit(0);
      });
    }
    //save obj
    else {
      Service.process(getString(R.string.saving), Service.SERVICE_SAVE, this, () -> {
        JNI.onToggleButtonClicked(false);
        mGLView.stop();
        finish();
        File input = new File(getTempPath(), "model" + Exporter.EXT_OBJ);
        if (JNI.save(input.getAbsolutePath().getBytes())) {
          Service.finish(input.getAbsolutePath());
        } else {
          Service.reset(Main.this);
          System.exit(0);
        }
      });
    }
  }

  private void showAndroidBugDialog() {
    runOnUiThread(() -> {
      if (Build.VERSION.SDK_INT >= 30) {
        AlertDialog.Builder dialog = new AlertDialog.Builder(Main.this);
        dialog.setTitle(R.string.app_name);
        dialog.setMessage(R.string.storage_bug);
        dialog.setPositiveButton(R.string.resolve_manually, (dialog1, which) -> openURL(this, "https://lvonasek.github.io/androidbug.html"));
        dialog.setNegativeButton(android.R.string.cancel, null);
        Dialog d = dialog.create();
        d.getWindow().setBackgroundDrawable(getDrawable(R.drawable.background_dialog));
        d.setOnDismissListener(dialog12 -> System.exit(0));
        d.show();
      }
    });
  }
}
