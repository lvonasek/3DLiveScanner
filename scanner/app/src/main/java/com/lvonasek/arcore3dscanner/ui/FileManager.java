package com.lvonasek.arcore3dscanner.ui;

import android.Manifest;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.net.Uri;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.text.Html;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.GridView;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.google.ar.core.ArCoreApk;
import com.lvonasek.arcore3dscanner.R;
import com.lvonasek.arcore3dscanner.main.Exporter;
import com.lvonasek.arcore3dscanner.main.Main;
import com.lvonasek.utils.Compatibility;
import com.lvonasek.utils.IO;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

public class FileManager extends AbstractActivity implements View.OnClickListener {
  private FileAdapter mAdapter;
  private GridView mList;
  private Button mAdd;
  private Button mCancel;
  private CheckBox mCheckbox;
  private ProgressBar mProgress;
  private TextView mText;
  private RelativeLayout mHeader;
  private LinearLayout mOptions;
  private TextView mName;
  private View mPosition;
  private View mRename;
  private View mShare;
  private static boolean allowedToAskForPermissions = true;

  @Override
  protected void onCreate(Bundle savedInstanceState)
  {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_files);

    boolean showPro = Compatibility.isPlayStoreSupported(this) && !isProVersion(this);
    findViewById(R.id.settings).setOnClickListener(this);

    mName = findViewById(R.id.name);
    mRename = findViewById(R.id.rename);
    mPosition = findViewById(R.id.position);
    mShare = findViewById(R.id.share);
    mHeader = findViewById(R.id.header);
    mOptions = findViewById(R.id.options);
    mPosition.setOnClickListener(this);
    mRename.setOnClickListener(this);
    mShare.setOnClickListener(this);
    findViewById(R.id.delete).setOnClickListener(this);

    mAdd = findViewById(R.id.add_button);
    mCancel = findViewById(R.id.service_cancel);
    mCheckbox = findViewById(R.id.checkbox);
    mList = findViewById(R.id.list);
    mText = findViewById(R.id.info_text);
    mProgress = findViewById(R.id.progressBar);
    mAdd.setOnClickListener(this);
    mCancel.setOnClickListener(this);

    int columns = 3;
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
    columns = pref.getInt(getString(R.string.pref_layout), columns);

    mAdapter = new FileAdapter(this, columns);
    mList.setOnTouchListener((view, event) -> {
      mAdapter.forwardTouch(event);
      return false;
    });
  }

  @Override
  public void onBackPressed()
  {
    if (mProgress.getVisibility() == View.VISIBLE) {
      System.exit(0);
    } else if (mAdapter.hasParent()) {
      mAdapter.toParent();
    } else if (mAdapter.getSelected() != null) {
      mAdapter.update();
    } else {
      moveTaskToBack(true);
    }
  }

  @Override
  public int getNavigationBarColor() {
        return Color.BLACK;
    }

  @Override
  public int getStatusBarColor() {
    return Color.argb(255, 48, 48, 48);
  }

  @Override
  protected void onResume()
  {
    super.onResume();
    mAdd.setVisibility(View.VISIBLE);
    mCancel.setVisibility(View.GONE);
    mProgress.setVisibility(View.GONE);

    int service = Service.getRunning(this);
    if (service > Service.SERVICE_NOT_RUNNING) {
      mAdd.setVisibility(View.GONE);
      mCancel.setVisibility(View.VISIBLE);
      mList.setVisibility(View.GONE);
      mText.setVisibility(View.VISIBLE);
      mText.setText("");
      new Thread(() -> {
        while(true) {
          try
          {
            Thread.sleep(1000);
          } catch (Exception e)
          {
            e.printStackTrace();
          }
          FileManager.this.runOnUiThread(() -> {
            if (Service.getMessage() == null)
              mText.setText(getString(R.string.failed));
            else
              mText.setText(getString(R.string.working) + "\n\n" + Service.getMessage());
          });
        }
      }).start();
    } else if (Service.getRunning(this) < Service.SERVICE_NOT_RUNNING)
    {
      service = Math.abs(Service.getRunning(this));
      mAdd.setVisibility(View.GONE);
      if (service != Service.SERVICE_SAVE) {
        mCancel.setVisibility(View.VISIBLE);
        mList.setVisibility(View.GONE);
        mText.setVisibility(View.VISIBLE);
      }
      boolean paused = service == Service.SERVICE_SAVE;
      int text = paused ? R.string.paused : R.string.finished;
      mText.setText(getString(text) + "\n" + getString(R.string.turn_off));
      if (service == Service.SERVICE_SAVE) {
        showProgress();
        startActivity(new Intent(this, Main.class));
      } else if ((service == Service.SERVICE_POSTPROCESS) || (service == Service.SERVICE_PHOTOGRAMMETRY)) {
        finishScanning();
      } else if (service == Service.SERVICE_SKETCHFAB) {
        Intent intent = new Intent(Intent.ACTION_VIEW);
        intent.setData(Uri.parse(Service.getLink(this)));
        startActivity(intent);
        Service.forceState(this, null, Service.SERVICE_NOT_RUNNING);
      }
    } else
      setupPermissions();
  }

  public void refreshUI()
  {
    String link = "https://lvonasek.github.io/policy-3dls.html";
    String info = getString(R.string.info).replaceAll("\n", "<br>");
    info = info.replaceAll("#BEGIN#", "<a href=" + link + ">").replaceAll("#END#", "</a>");
    info = info.replaceAll("\"Models\"", "\"Documents\\\\3D Live Scanner\"");

    AlertDialog d;
    String policyKey = "KEY_POLICY_ACCEPTED";
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
    if (!pref.getBoolean(policyKey, false) && !Compatibility.isPlayStoreSupported(this)) {
      mAdd.setVisibility(View.GONE);
      mCancel.setVisibility(View.VISIBLE);
      mCheckbox.setVisibility(View.VISIBLE);
      mCheckbox.setOnCheckedChangeListener((buttonView, isChecked) -> mCancel.setBackgroundResource(isChecked ? R.drawable.background_button : R.drawable.background_button_selected));
      mList.setVisibility(View.GONE);
      mCancel.setBackgroundResource(R.drawable.background_button_selected);
      mCancel.setText(android.R.string.ok);
      mCancel.setOnClickListener(view -> {
        if (mCheckbox.isChecked()) {
          SharedPreferences.Editor e = pref.edit();
          e.putBoolean(policyKey, true);
          e.apply();
          refreshUI();
        }
      });
      mText.setText(Html.fromHtml(info, Html.FROM_HTML_MODE_LEGACY));
      mText.setOnClickListener(v -> openURL(FileManager.this, link));
      return;
    } else if (Initializator.isFirst() && Compatibility.isPlayStoreSupported(this)) {
      LayoutInflater inflater = (LayoutInflater)getSystemService(Context.LAYOUT_INFLATER_SERVICE);
      View view = inflater.inflate(R.layout.dialog_start, null);

      TextView text = view.findViewById(R.id.info);
      text.setText(Html.fromHtml(info, Html.FROM_HTML_MODE_LEGACY));
      text.setOnClickListener(v -> openURL(FileManager.this, link));

      AlertDialog.Builder dialog = new AlertDialog.Builder(this);
      dialog.setView(view);

      d = dialog.create();
      d.getWindow().setBackgroundDrawable(getDrawable(R.drawable.background_dialog));
      d.show();

      if (!Compatibility.isGoogleDepthSupported(this) && !Compatibility.hasToFSensor(this)) {
        d.findViewById(R.id.lowend_device).setVisibility(View.VISIBLE);
      }
    }

    long time = System.currentTimeMillis();
    boolean migrate = hasFilesToMigrate(this);
    if (migrate) {
      Log.d(TAG, "Some files has to be migrated");
    }
    mCancel.setVisibility(View.GONE);
    mCheckbox.setVisibility(View.GONE);
    mList.setVisibility(View.VISIBLE);
    mText.setOnClickListener(null);
    mText.setText(migrate ? R.string.migrating_data : R.string.wait);
    mText.setVisibility(mAdapter.isEmpty() ? View.VISIBLE : View.GONE);
    new Thread(() -> {

      //update file structure
      Exporter.makeStructure(getPath(migrate));

      //get list of files
      runOnUiThread(() -> {
        mAdapter.update();
        Log.d(TAG, "Listing files took " + (System.currentTimeMillis() - time) + "ms");

        mText.setText(R.string.no_data);
        mText.setVisibility(mAdapter.getCount() == 0 ? View.VISIBLE : View.GONE);
        mList.setAdapter(mAdapter);
        mAdd.setVisibility(View.VISIBLE);
        mProgress.setVisibility(View.GONE);

        mAdapter.notifyDataSetChanged();
        if (mAdapter.getCount() > 0) {
          mList.setSelection(0);
        }
      });
    }).start();
  }

  protected void setupPermissions() {
    String[] permissions = {
            Manifest.permission.CAMERA,
            Manifest.permission.INTERNET
    };

    boolean ok = true;
    for (String s : permissions)
      if (checkSelfPermission(s) != PackageManager.PERMISSION_GRANTED)
        ok = false;

    if (!allowedToAskForPermissions && !ok) {
      mAdd.setVisibility(View.GONE);
      mCancel.setVisibility(View.VISIBLE);
      mList.setVisibility(View.GONE);
      mCancel.setText(android.R.string.ok);
      mCancel.setOnClickListener(view -> {
        allowedToAskForPermissions = true;
        setupPermissions();
      });
      mText.setText(R.string.permissions_required);
      return;
    } else {
      mAdd.setVisibility(View.VISIBLE);
      mList.setVisibility(View.VISIBLE);
      mCancel.setText(android.R.string.cancel);
      mCancel.setOnClickListener(this);
      mCancel.setVisibility(View.GONE);
      allowedToAskForPermissions = false;
    }

    try {
      boolean arcore = Compatibility.isPlayStoreSupported(this);
      boolean arengine = Compatibility.shouldUseHuawei(this);
      if ((!arengine || arcore) && Compatibility.isARSupported(this))
        if (ArCoreApk.getInstance().requestInstall(this, true) != ArCoreApk.InstallStatus.INSTALLED)
          return;
    } catch (Exception e) {
      e.printStackTrace();
    }

    long timestamp = System.currentTimeMillis();
    onPermissionFail = () -> {
      if (System.currentTimeMillis() - timestamp < 100) {
        Intent intent = new Intent(android.provider.Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
        Uri uri = Uri.fromParts("package", getPackageName(), null);
        intent.setData(uri);
        startActivity(intent);
      }
    };
    onPermissionSuccess = () -> {
      if (Initializator.hasFileIntent()) {
        showProgress();

        new Thread(() -> {

          int index = 0;
          File path;
          do {
            index++;
            path = new File(getPath(false), "Import_" + index + ".obj");
          } while (path.exists());
          path.mkdirs();

          boolean success = IO.unzip(path.getAbsolutePath() + "/", Initializator.getFile(FileManager.this));
          File finalPath = path;
          runOnUiThread(() -> {
            if (success) {
              Intent intent = new Intent(FileManager.this, Main.class);
              intent.putExtra(AbstractActivity.FILE_KEY, finalPath.getAbsolutePath());
              startActivity(intent);
            } else {
              refreshUI();
            }
          });
        }).start();
      } else {
        refreshUI();
      }
    };
    askForPermissions(permissions);
  }

  public void showProgress()
  {
    try {
      mAdd.setVisibility(View.GONE);
      mProgress.setVisibility(View.VISIBLE);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void onClick(View v) {
    int id = v.getId();

    if (id == R.id.delete) {
      mAdapter.deleteModel();
    } else if (id == R.id.position) {
      mAdapter.showPosition();
    } else if (id == R.id.rename) {
      mAdapter.rename();
    } else if (id == R.id.share) {
      mAdapter.shareModel();
    } else if (id == R.id.add_button) {
      SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
      if (pref.getBoolean(getString(R.string.pref_gps), false)) {
        String[] permissions = {
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION
        };
        onPermissionSuccess = this::startScanning;
        askForPermissions(permissions);
      } else {
        startScanning();
      }
    } else if (id == R.id.service_cancel) {
      Service.reset(this);
      System.exit(0);
    } else if (id == R.id.settings) {
      startActivity(new Intent(this, Settings.class));
    }
  }


  private void startScanning()
  {
    AlertDialog.Builder builder = new AlertDialog.Builder(this);
    builder.setView(R.layout.dialog_scan);
    Dialog dialog = builder.create();
    dialog.getWindow().setBackgroundDrawable(getDrawable(R.drawable.background_dialog));
    dialog.show();

    ArrayList<Drawable> icons = new ArrayList<>();
    ArrayList<String> values = new ArrayList<>();
    if (Compatibility.isARSupported(this)) {
      icons.add(getDrawable(R.drawable.ic_type_face));
      values.add(getString(R.string.mode_face));
      icons.add(getDrawable(R.drawable.ic_type_scan));
      values.add(getString(R.string.mode_realtime));
      if (isProVersion(this)) {
        icons.add(getDrawable(R.drawable.ic_type_dataset));
        values.add(getString(R.string.mode_dataset));
      }
    }

    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(FileManager.this);
    ArrayAdapterWithIcons adapter = new ArrayAdapterWithIcons(this, values, icons);
    GridView list = dialog.findViewById(R.id.list);
    list.setAdapter(adapter);
    list.setOnTouchListener((v, event) -> event.getAction() == MotionEvent.ACTION_MOVE);
    list.setOnItemClickListener((adapterView, view, index, l) -> {
      dialog.dismiss();
      showProgress();

      String mode = values.get(index);
      SharedPreferences.Editor e = pref.edit();
      if (mode.compareTo(getString(R.string.mode_dataset)) == 0) {
        e.putBoolean(getString(R.string.pref_later), true);
        e.putString(getString(R.string.pref_mode), "realtime");
      } else if (mode.compareTo(getString(R.string.mode_face)) == 0) {
        e.putBoolean(getString(R.string.pref_later), false);
        e.putString(getString(R.string.pref_mode), "face");
      } else if (mode.compareTo(getString(R.string.mode_realtime)) == 0) {
        e.putBoolean(getString(R.string.pref_later), false);
        e.putString(getString(R.string.pref_mode), "realtime");
      }
      e.commit();

      startActivity(new Intent(FileManager.this, Main.class));
    });
  }

  private void finishScanning()
  {
    mCancel.setVisibility(View.GONE);
    showProgress();
    Date date = new Date() ;
    SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);
    final String filename = dateFormat.format(date);
    String text = getString(R.string.data_saved) + " " + filename;
    Toast.makeText(this, text, Toast.LENGTH_LONG).show();

    new Thread(() -> {
      File file = new File(Service.getLink(FileManager.this));
      File file2save = Exporter.export(file, filename);

      //remove temp dir
      if (!isPostProcessLaterOn(FileManager.this))
        deleteRecursive(new File(file.getParent()));

      //finish
      Service.reset(FileManager.this);
      Intent intent = new Intent(FileManager.this, Main.class);
      intent.putExtra(FILE_KEY, file2save.getAbsolutePath());
      showProgress();
      startActivity(intent);
    }).start();
  }

  public void setColumns(int count) {
    mList.setNumColumns(count);

    SharedPreferences.Editor e = PreferenceManager.getDefaultSharedPreferences(this).edit();
    e.putInt(getString(R.string.pref_layout), count);
    e.commit();
  }

  public void setOptions(int size) {
    boolean on = size > 0;
    mHeader.setVisibility(on ? View.INVISIBLE : View.VISIBLE);
    mOptions.setVisibility(on ? View.VISIBLE : View.GONE);

    if (on) {
      mName.setText(mAdapter.getSelected());
    }

    boolean more = size > 1;
    boolean ext = mAdapter.hasExtension();
    mPosition.setVisibility(!more && mAdapter.hasPosition() ? View.VISIBLE : View.GONE);
    mRename.setVisibility(!more ? View.VISIBLE : View.GONE);
    mShare.setVisibility(ext && !more ? View.VISIBLE : View.GONE);

    int background = Color.argb(128, 0, 153, 204);
    setWindow(on ? background : getStatusBarColor(), getNavigationBarColor());
  }
}
