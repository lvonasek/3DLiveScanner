package com.lvonasek.arcore3dscanner.ui;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.content.res.Resources;
import android.graphics.Color;
import android.net.Uri;
import android.os.Environment;
import android.os.Parcelable;
import android.preference.PreferenceManager;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;

import com.lvonasek.arcore3dscanner.main.Exporter;
import com.lvonasek.utils.Compass;
import com.lvonasek.utils.Compatibility;
import com.lvonasek.utils.IO;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class AbstractActivity extends Activity {
  protected static final String DELETE_POSTFIX = ".#$%";
  protected static final String FILE_KEY = "FILE2OPEN";
  protected static final String TEMP_DIRECTORY = "dataset";
  protected static final String URL_KEY = "URL2OPEN";

  private static final String NEW_MODEL_DIRECTORY = "/3D Live Scanner/";
  private static final String OLD_MODEL_DIRECTORY = "/Models/";
  private static final int PERMISSIONS_CODE = 1987;
  public static final String TAG = "arcore_app";

  private Compass mCompass;
  protected Runnable onPermissionFail = null;
  protected Runnable onPermissionSuccess = null;
  private static final ArrayList<File> toDelete = new ArrayList<>();
  private static final AtomicBoolean migrationActive = new AtomicBoolean(false);
  private static final AtomicBoolean restartApp = new AtomicBoolean(false);

  public float convertDpToPx(float dp) {
    return dp * ((float) getResources().getDisplayMetrics().densityDpi / DisplayMetrics.DENSITY_DEFAULT);
  }

  public float convertPxToDp(float px) {
    return px / ((float) getResources().getDisplayMetrics().densityDpi / DisplayMetrics.DENSITY_DEFAULT);
  }

  public static void deleteOnBackground(File file) {
    synchronized (toDelete) {

      //add file into queue
      boolean start = toDelete.isEmpty();
      toDelete.add(file);

      //start background thread
      if (start) {
        new Thread(() -> {
          while (true) {

            //get next file
            File f;
            synchronized (toDelete) {
              if (toDelete.isEmpty()) {
                return;
              }
              f = toDelete.get(0);
              toDelete.remove(0);
            }

            //delete file
            IO.deleteRecursive(f);
          }
        }).start();
      }
    }
  }

  public static void deleteRecursive(File file) {
    for (int i = 0; i < 50; i++) {
      File finalFile = new File(file.getAbsolutePath() + DELETE_POSTFIX + i);
      if (file.renameTo(finalFile)) {
        deleteOnBackground(finalFile);
        return;
      }
    }
    IO.deleteRecursive(file);
  }

  public int getNavigationBarHeight() {
    Resources resources = getResources();
    int resourceId = resources.getIdentifier("navigation_bar_height", "dimen", "android");
    if (resourceId > 0) {
      return resources.getDimensionPixelSize(resourceId);
    }
    return 0;
  }

  public int getStatusBarHeight() {
    int result = 0;
    int resourceId = getResources().getIdentifier("status_bar_height", "dimen", "android");
    if (resourceId > 0) {
      result = getResources().getDimensionPixelSize(resourceId);
    }
    return result;
  }

  public abstract int getNavigationBarColor();

  public abstract int getStatusBarColor();

  public static int getBackend(Activity context)
  {
    return Compatibility.shouldUseHuawei(context) ? 1 : 0;
  }

  public static float getResolution(Context context)
  {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
    String value = pref.getString(context.getString(com.lvonasek.arcore3dscanner.R.string.pref_resolution), "0.04");
    if (value.compareTo("0") == 0) value = "0.04";
    return Float.parseFloat(value);
  }

  public static boolean isCameraFeedOn(Context context) {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
    return pref.getBoolean(context.getString(com.lvonasek.arcore3dscanner.R.string.pref_camera), false);
  }

  public static boolean isFaceModeOn(Context context)
  {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
    return pref.getString(context.getString(com.lvonasek.arcore3dscanner.R.string.pref_mode), "realtime").compareTo("face") == 0;
  }

  public static boolean isProVersion(Context context) {
    return true;
  }

  public static boolean isPostProcessLaterOn(Context context) {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
    boolean later = pref.getBoolean(context.getString(com.lvonasek.arcore3dscanner.R.string.pref_later), false);
    String mode = pref.getString(context.getString(com.lvonasek.arcore3dscanner.R.string.pref_mode), "realtime");
    return later || (mode.compareTo("dataset") == 0);
  }

  public static boolean isTofOn(Activity activity)
  {
    boolean supported = isTofSupported(activity);
    String key = activity.getString(com.lvonasek.arcore3dscanner.R.string.pref_depth);
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(activity);
    return pref.getBoolean(key, supported) && supported;
  }

  public static boolean isTofSupported(Activity activity)
  {
    switch (getBackend(activity)) {
      case 0:
        return Compatibility.isGoogleToFSupported(activity);
      case 1:
        return Compatibility.isHuaweiToFSupported(activity);
      default:
        return false;
    }
  }

  public static ArrayList<String> listFiles(File file) {
    ArrayList<String> output = new ArrayList<>();
    File[] files = file.listFiles();
    if (files != null) {
      for (File f : files) {
        if (!f.getAbsolutePath().contains(DELETE_POSTFIX)) {
          output.add(f.getAbsolutePath());
        } else {
          deleteOnBackground(f);
        }
      }
    }
    return output;
  }

  public static void openURL(Activity context, String url) {

    //get apps capable of opening URL
    Intent intent = new Intent(Intent.ACTION_VIEW);
    intent.setData(Uri.parse(url));
    List<Intent> targetedShareIntents = new ArrayList<>();
    for (ResolveInfo info : context.getPackageManager().queryIntentActivities(intent, 0)) {
      Intent targetedShare = new Intent(Intent.ACTION_VIEW);
      targetedShare.setData(Uri.parse(url));
      if (!info.activityInfo.packageName.equalsIgnoreCase(context.getPackageName())) {
        targetedShare.setPackage(info.activityInfo.packageName);
        targetedShareIntents.add(targetedShare);
      }
    }

    //show chooser
    if (!targetedShareIntents.isEmpty()) {
      Intent chooserIntent = Intent.createChooser(targetedShareIntents.remove(0), url);
      chooserIntent.putExtra(Intent.EXTRA_INITIAL_INTENTS, targetedShareIntents.toArray(new Parcelable[0]));
      context.startActivity(chooserIntent);
    }
  }

  public static void setOrientation(boolean portrait, Activity activity) {
    int value = ActivityInfo.SCREEN_ORIENTATION_PORTRAIT;
    if (!portrait)
      value = ActivityInfo.SCREEN_ORIENTATION_SENSOR_LANDSCAPE;
    activity.setRequestedOrientation(value);
  }

  @Override
  protected void onPause() {
    mCompass.onPause();
    super.onPause();
  }

  @Override
  protected void onResume() {
    super.onResume();
    setWindow(getStatusBarColor(), getNavigationBarColor());
    setOrientation(true, this);

    mCompass = new Compass(this);
    mCompass.onResume();
  }

  public void setWindow(int statusBarColor, int navigationBarColor) {
    int lFlags = getWindow().getDecorView().getSystemUiVisibility();
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    getWindow().setStatusBarColor(statusBarColor);
    getWindow().setNavigationBarColor(navigationBarColor);
    if (Color.red(navigationBarColor) > 128)
      getWindow().getDecorView().setSystemUiVisibility(lFlags | View.SYSTEM_UI_FLAG_LIGHT_STATUS_BAR);
    else
      getWindow().getDecorView().setSystemUiVisibility(lFlags & ~View.SYSTEM_UI_FLAG_LIGHT_STATUS_BAR);
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
    if (requestCode == PERMISSIONS_CODE) {
      for (int r : grantResults) {
        if (r != PackageManager.PERMISSION_GRANTED) {
          if (onPermissionFail != null) {
            onPermissionFail.run();
          }
          onPermissionFail = null;
          onPermissionSuccess = null;
          return;
        }
      }
      if (onPermissionSuccess != null) {
        onPermissionSuccess.run();
      }
      onPermissionSuccess = null;
      onPermissionFail = null;
    }
    else
    {
      super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    }
  }

  protected void askForPermissions(String[] permissions) {
    boolean ok = true;
    for (String s : permissions)
      if (checkSelfPermission(s) != PackageManager.PERMISSION_GRANTED)
        ok = false;

    if (!ok)
      requestPermissions(permissions, PERMISSIONS_CODE);
    else
      onRequestPermissionsResult(PERMISSIONS_CODE, null, new int[]{PackageManager.PERMISSION_GRANTED});
  }

  public static File getModel(File folder) {
    ArrayList<String> files = AbstractActivity.listFiles(folder);
    if (folder.getAbsolutePath().endsWith(Exporter.EXT_OBJ)) {
      for (String s : files) {
        File f = new File(s);
        if (f.getAbsolutePath().toLowerCase().endsWith(Exporter.EXT_OBJ))
          return f;
      }
      return folder;
    }
    if (folder.getAbsolutePath().endsWith(Exporter.EXT_PLY)) {
      for (String s : files) {
        File f = new File(s);
        if (f.getAbsolutePath().toLowerCase().endsWith(Exporter.EXT_PLY))
          return f;
        return folder;
      }
    }
    return null;
  }

  public static String getPath(boolean migrate) {
    String olddir = Environment.getExternalStorageDirectory().getPath() + OLD_MODEL_DIRECTORY;
    String newdir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS).getPath() + OLD_MODEL_DIRECTORY;
    String dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS).getPath() + NEW_MODEL_DIRECTORY;

    if (new File(dir).mkdir())
      Log.d(TAG, "Directory " + dir + " created");
    try {
      if (new File(new File(dir), ".nomedia").createNewFile())
        Log.d(TAG, ".nomedia in  " + dir + " created");
    } catch (Exception e) {
      e.printStackTrace();
    }

    if (migrate) {
      synchronized (migrationActive) {
        migrationActive.set(true);
      }
      migrate(olddir, dir);
      migrate(newdir, dir);
      synchronized (migrationActive) {
        migrationActive.set(false);
        if (restartApp.get()) {
          System.exit(0);
        }
      }
    }
    return dir;
  }

  public static File getTempPath() {
    File dir = new File(getPath(false), TEMP_DIRECTORY);
    if (dir.mkdir())
      Log.d(TAG, "Directory " + dir + " created");
    return dir;
  }

  public static boolean hasFilesToMigrate(Context context) {
    String key = "MIGRATION_DONE";
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
    if (pref.getBoolean(key, false)) {
      return false;
    }
    SharedPreferences.Editor e = pref.edit();
    e.putBoolean(key, true);
    e.commit();

    String olddir = Environment.getExternalStorageDirectory().getPath() + OLD_MODEL_DIRECTORY;
    String newdir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS).getPath() + OLD_MODEL_DIRECTORY;
    if (new File(olddir).exists()) {
      File[] files = new File(olddir).listFiles();
      if ((files != null) && (files.length > 0)) {
        return true;
      }
    }
    if (new File(newdir).exists()) {
      File[] files = new File(newdir).listFiles();
      if ((files != null) && (files.length > 0)) {
        return true;
      }
    }
    return false;
  }

  private static void migrate(String olddir, String newdir) {
    Log.d(TAG, "Migrating " + olddir + " into " + newdir);
    if (new File(olddir).exists()) {
      boolean ok = true;
      File[] files = new File(olddir).listFiles();
      if (files != null) {
        for (File file : files) {
          if (file.renameTo(new File(newdir, file.getName()))) {
            Log.d(TAG, file.getName() + " migrated");
          } else {
            Log.d(TAG, "Unable to migrate " + file.getName());
            ok = false;
          }
        }
      }
      if (ok) {
        if (new File(olddir).delete()) {
          Log.d(TAG, "Directory " + olddir + " deleted");
        }
      }
    }
  }
}
