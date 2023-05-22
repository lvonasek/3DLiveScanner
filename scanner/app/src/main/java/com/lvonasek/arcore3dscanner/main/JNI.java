package com.lvonasek.arcore3dscanner.main;

import android.content.Context;
import android.content.res.Resources;

import com.lvonasek.arcore3dscanner.R;

/**
 * Interfaces between native C++ code and Java code.
 */
public class JNI
{
  static {
    System.loadLibrary("3dscanner");
  }

  public static boolean motionTrackingMessages = true;

  // Called when the Tango service is connected successfully.
  public static native boolean onARServiceConnected(Context context, double res, double dmin,
                                                    double dmax, int noise, boolean holes,
                                                    boolean poseCorr, boolean distortion, boolean offset,
                                                    boolean flashlight, int mode, boolean clearing, byte[] temp);

  // Setup the view port width and height.
  public static native void onGlSurfaceChanged(int width, int height, boolean fullhd);

  // Main render loop.
  public static native boolean onGlSurfaceDrawFrame(boolean faceMode, float yaw, int viewMode, boolean anchors, boolean grid, boolean smooth);

  // Called when the toggle button is clicked
  public static native void onToggleButtonClicked(boolean reconstructionRunning);

  // Called when the clear button is clicked
  public static native void onClearButtonClicked();

  // Called when the undo button is clicked
  public static native void onUndoButtonClicked(boolean fromUser, boolean texturize);

  // Reload the model preview from dataset
  public static native void onUndoPreviewUpdate(int frames);

  // Pauses the AR
  public static native void onPause();

  // Extract data from the dataset
  public static native void extract(byte[] path, int mode);

  // Load 3D model from file
  public static native boolean load(byte[] name);

  // Optimize 3D model (object's only)
  public static native void optimize(byte[] name);

  // Save current 3D model
  public static native boolean save(byte[] name);

  // Save current 3D model with textures (editor usage)
  public static native void saveWithTextures(byte[] name);

  // Set parameters of texturing
  public static native void setTextureParams(int detail, int res, int count);

  // Texturize 3D model
  public static native void texturize(byte[] input, byte[] output, boolean poisson, boolean twoPass);

  // Set view on 3D view
  public static native void setView(float pitch, float yaw, float x, float y, float z, float o, boolean gyro);

  // Count distance between two points on screen
  public static native float getDistance(float x1, float y1, float x2, float y2);

  // Detect floor level for position
  public static native float getFloorLevel(float x, float y, float z);

  // Gets position of specified axis
  public static native float getView(int axis);

  // Get system event
  private static native byte[] getEvent();

  // Scan size
  public static native int getScanSize();

  // Get back previous state of the model
  public static native void restore();

  // Enable or disable photo mode
  public static native void setPhotoMode(boolean on);

  // Apply effect on model
  public static native void applyEffect(int effect, float value, int axis);

  // Preview effect on model
  public static native void previewEffect(int effect, float value, int axis);

  // Enable/disable showing normals
  public static native void showNormals(boolean on);

  // Apply select on model
  public static native void applySelect(float x, float y, boolean triangle);

  // Select or deselect all
  public static native void completeSelection(boolean inverse);

  // Increase or decrease selection
  public static native void multSelection(boolean increase);

  // Select object by circle
  public static native void circleSelection(float x, float y, float radius, boolean invert);

  // Select object by rect
  public static native void rectSelection(float x1, float y1, float x2, float y2, boolean invert);

  // Indicate that last setview was applied
  public static native boolean animFinished();

  // Indicate that the motion tracking jumped
  public static native boolean didARjump();

  public static String getEvent(Resources r)
  {
    String event = new String(getEvent());
    event = event.replace("ANALYSE", r.getString(R.string.event_analyse));
    event = event.replace("FEW_FEATURES", r.getString(R.string.event_features));
    event = event.replace("CONVERT", r.getString(R.string.event_convert));
    event = event.replace("IMAGE", r.getString(R.string.event_image));
    event = event.replace("MERGE", r.getString(R.string.event_merge));
    event = event.replace("PROCESS", r.getString(R.string.event_process));
    event = event.replace("SIMPLIFY", r.getString(R.string.event_simplify));
    event = event.replace("UNWRAP", r.getString(R.string.event_unwrap));
    event = event.replace("POISSON", r.getString(R.string.poisson));
    event = event.replace("ALIGNMENT", r.getString(R.string.align_pose));

    if (motionTrackingMessages) {
      event = event.replace("MT_INIT", "");
      event = event.replace("MT_JUMP", r.getString(R.string.jump));
      event = event.replace("MT_LOST", r.getString(R.string.event_features) + "\n" + r.getString(R.string.move));
    } else {
      if (event.startsWith("MT_"))
        event = "";
    }
    return event;
  }
}
