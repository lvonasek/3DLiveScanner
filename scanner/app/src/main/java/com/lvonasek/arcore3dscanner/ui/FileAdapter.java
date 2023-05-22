package com.lvonasek.arcore3dscanner.ui;

import android.app.AlertDialog;
import android.app.Dialog;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.net.Uri;
import android.os.Build;
import android.preference.PreferenceManager;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.GridView;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.core.content.FileProvider;

import com.lvonasek.arcore3dscanner.BuildConfig;
import com.lvonasek.arcore3dscanner.R;
import com.lvonasek.arcore3dscanner.main.Exporter;
import com.lvonasek.arcore3dscanner.main.Main;
import com.lvonasek.arcore3dscanner.sketchfab.OAuth;
import com.lvonasek.utils.GestureDetector;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Locale;
import java.util.Scanner;

class FileAdapter extends BaseAdapter
{
  private FileManager mContext;
  private String mPath;
  private ArrayList<Integer> mSelected;
  private GestureDetector mGesture;
  private float mColumns;

  private final HashMap<String, Drawable> mIcons = new HashMap<>();
  private final ArrayList<String> mItems = new ArrayList<>();

  FileAdapter(FileManager context, int columns)
  {
    mContext = context;
    mColumns = columns;
    mPath = AbstractActivity.getPath(false);
    mSelected = new ArrayList<>();

    mContext.setColumns(columns);
    mGesture = new GestureDetector(new GestureDetector.GestureListener() {
      @Override
      public boolean IsAcceptingRotation() {
        return false;
      }

      @Override
      public void OnDrag(float dx, float dy) {
      }

      @Override
      public void OnTwoFingerMove(float dx, float dy) {
      }

      @Override
      public void OnTwoFingerRotation(float angle) {
      }

      @Override
      public void OnPinchToZoom(float diff) {
        int before = (int)mColumns;
        mColumns -= diff * 0.75f;
        if (mColumns < 2) {
          mColumns = 2;
        }
        if (mColumns > 5) {
          mColumns = 5;
        }
        int after = (int)mColumns;

        if (before != after) {
          mContext.setColumns(after);
        }
      }
    }, mContext);
  }

  @Override
  public int getCount()
  {
    return mItems.size();
  }

  @Override
  public Object getItem(int i)
  {
    return mItems.get(i);
  }

  @Override
  public long getItemId(int i)
  {
    return i;
  }

  @Override
  public View getView(final int index, View view, ViewGroup viewGroup)
  {
    LayoutInflater inflater = (LayoutInflater) mContext.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
    view = inflater.inflate(R.layout.view_item, null, true);
    if (getCount() <= index) {
      return view;
    }
    String key = (String)getItem(index);
    TextView name = view.findViewById(R.id.name);
    if (key.lastIndexOf('.') > 0) {
      name.setText(key.substring(0, key.lastIndexOf('.')));
    } else {
      name.setText(key);
    }

    //set icon
    boolean hasExtension = false;
    ImageView icon = view.findViewById(R.id.icon);
    icon.setImageDrawable(mContext.getDrawable(R.drawable.ic_folder));
    if (key.compareTo(mContext.getString(R.string.folder_up)) == 0) {
      icon.setImageDrawable(mContext.getDrawable(R.drawable.ic_folder_up));
    }
    for (String ext : Exporter.FILE_EXT) {
      if (key.endsWith(ext)) {
        icon.setImageDrawable(mContext.getDrawable(R.drawable.ic_model_icon));
        hasExtension = true;
        break;
      }
    }
    synchronized (mIcons) {
      if (mIcons.containsKey(key)) {
        icon.setImageDrawable(mIcons.get(key));
      }
    }
    loadIcon(key, icon);

    //set extension
    View extension = view.findViewById(R.id.extension);
    extension.setVisibility(key.endsWith(Exporter.EXT_DATASET) ? View.VISIBLE : view.GONE);

    //set selection
    View selection = view.findViewById(R.id.selection);
    if (mSelected.contains(index)) {
      selection.setVisibility(View.VISIBLE);
    }

    //set open action
    boolean finalHasExtension = hasExtension;
    view.setOnClickListener(v -> {
      if (!mSelected.isEmpty()) {
        setSelected(selection, index);
        return;
      }

      if (!finalHasExtension) {
        if (hasParent() && (index == 0)) {
          mPath = new File(mPath).getParentFile().getAbsolutePath();
        } else {
          mPath = new File(mPath, key).getAbsolutePath();
        }
        mContext.refreshUI();
      } else if (key.endsWith(Exporter.EXT_DATASET)) {
        startPostprocess(key);
      } else {
        File f = new File(getPath(), key);
        Intent intent = new Intent(mContext, Main.class);
        intent.putExtra(AbstractActivity.FILE_KEY, f.getAbsolutePath());
        mContext.showProgress();
        mContext.startActivity(intent);
      }
    });

    view.setOnLongClickListener(view1 -> {
      setSelected(selection, index);
      return true;
    });

    return view;
  }

  public boolean hasParent() {
    return (getPath() + "/").compareTo(AbstractActivity.getPath(false)) != 0;
  }

  public void toParent() {
    mPath = new File(mPath).getParentFile().getAbsolutePath();
    mContext.refreshUI();
  }

  private void loadIcon(String name, ImageView view) {
    new Thread(() -> {
      try {
        File thumbFile = null;
        if (name.endsWith(Exporter.EXT_DATASET)) {
          thumbFile = new File(getPath(), name + "/thumbnail.jpg");
          if (!thumbFile.exists()) {
            File file = new File(getPath(), name + "/00000000.jpg");
            if (file.exists()) {
              Bitmap originalBitmap = BitmapFactory.decodeFile(file.getAbsolutePath());;
              Bitmap resizedBitmap = Bitmap.createScaledBitmap(originalBitmap, 180, 320, false);
              try (FileOutputStream out = new FileOutputStream(thumbFile.getAbsolutePath())) {
                resizedBitmap.compress(Bitmap.CompressFormat.JPEG, 75, out);
              } catch (Exception e) {
                e.printStackTrace();
              }
            }
          }
        } else if (name.endsWith(Exporter.EXT_OBJ)) {
          thumbFile = new File(getPath(), name + "/thumbnail.jpg");
          if (!thumbFile.exists()) {
            File f = new File(getPath(), name);
            if (f.exists()) {
              File model = AbstractActivity.getModel(f);
              String mtl = Exporter.getMtlResource(model.getAbsolutePath());
              File file = new File(model.getParent(), mtl + ".png");
              if (file.exists()) {
                Bitmap originalBitmap = BitmapFactory.decodeFile(file.getAbsolutePath());;
                Bitmap resizedBitmap = Bitmap.createScaledBitmap(originalBitmap, 256, 256, false);
                try (FileOutputStream out = new FileOutputStream(thumbFile.getAbsolutePath())) {
                  resizedBitmap.compress(Bitmap.CompressFormat.JPEG, 75, out);
                } catch (Exception e) {
                  e.printStackTrace();
                }
              }
            }
          }
        }
        if (thumbFile != null && thumbFile.exists() && thumbFile.length() >= 1024) {
          Bitmap bitmap = BitmapFactory.decodeFile(thumbFile.getAbsolutePath());
          int w = bitmap.getWidth();
          int h = bitmap.getHeight();
          int o = (Math.max(w, h) - Math.min(w, h)) / 2;
          if (w < h) {
            bitmap = Bitmap.createBitmap(bitmap, 0, o, w, w);
          } else if (w > h) {
            bitmap = Bitmap.createBitmap(bitmap, o, 0, h, h);
          }
          if ((bitmap.getWidth() != 256) || (bitmap.getHeight() != 256)) {
            bitmap = Bitmap.createScaledBitmap(bitmap, 256, 256, true);
          }
          Drawable d = new BitmapDrawable(mContext.getResources(), bitmap);
          synchronized (mIcons) {
            if (mIcons.containsKey(name)) {
              mIcons.remove(name);
            }
            mIcons.put(name, d);
          }
          mContext.runOnUiThread(() -> view.setImageDrawable(d));
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }).start();
  }

  public String getPath() {
    return new File(mPath).getAbsolutePath();
  }

  private void startPostprocess(String key) {

    AlertDialog.Builder builder = new AlertDialog.Builder(mContext);
    builder.setView(R.layout.dialog_scan);
    Dialog dialog = builder.create();
    dialog.getWindow().setBackgroundDrawable(mContext.getDrawable(R.drawable.background_dialog));
    dialog.show();
    ((TextView)dialog.findViewById(R.id.name)).setText(R.string.export);


    ArrayList<Drawable> icons = new ArrayList<>();
    ArrayList<String> values = new ArrayList<>();
    values.add(mContext.getString(R.string.export_model));
    values.add(mContext.getString(R.string.export_floorplan));
    values.add(mContext.getString(R.string.export_pointcloud));
    icons.add(mContext.getDrawable(R.drawable.ic_type_scan));
    icons.add(mContext.getDrawable(R.drawable.ic_type_floorplan));
    icons.add(mContext.getDrawable(R.drawable.ic_type_pointcloud));

    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(mContext);
    ArrayAdapterWithIcons adapter = new ArrayAdapterWithIcons(mContext, values, icons);
    GridView list = dialog.findViewById(R.id.list);
    list.setAdapter(adapter);
    list.setOnTouchListener((v, event) -> event.getAction() == MotionEvent.ACTION_MOVE);
    list.setOnItemClickListener((adapterView, view, index, l) -> {
      String mode = values.get(index);
      SharedPreferences.Editor e = pref.edit();
      e.putBoolean(mContext.getString(R.string.pref_later), true);
      if (mode.compareTo(mContext.getString(R.string.export_floorplan)) == 0) {
        e.putString(mContext.getString(R.string.pref_mode), "exp_floorplan");
      } else if (mode.compareTo(mContext.getString(R.string.export_pointcloud)) == 0) {
        e.putString(mContext.getString(R.string.pref_mode), "exp_pointcloud");
      } else if (mode.compareTo(mContext.getString(R.string.export_model)) == 0) {
        e.putString(mContext.getString(R.string.pref_mode), "realtime");
      }
      e.commit();

      File file = new File(getPath(), key);
      Intent intent = new Intent(mContext, Main.class);
      intent.putExtra(AbstractActivity.FILE_KEY, file.getAbsolutePath());
      dialog.dismiss();
      mContext.startActivity(intent);
    });
  }

  public void update() {
    mItems.clear();
    mSelected.clear();
    mContext.setOptions(mSelected.size());

    if (hasParent()) {
      mItems.add(mContext.getString(R.string.folder_up));
    }

    String[] files = new File(getPath()).list();
    if (files != null) {
      Arrays.sort(files);

      ArrayList<String> folders = new ArrayList<>();
      for (String s : files) {
        if (s.contains(AbstractActivity.DELETE_POSTFIX)) {
          continue;
        }
        File f = new File(getPath(), s);
        if (f.isDirectory()) {
          if (f.getAbsolutePath().compareTo(mContext.getTempPath().getAbsolutePath()) != 0) {
            if (Exporter.isFolder(s)) {
              folders.add(s);
            }
          }
        }
      }

      ArrayList<String> data = new ArrayList<>();
      for (String s : files) {
        if (s.contains(AbstractActivity.DELETE_POSTFIX)) {
          continue;
        }
        if (Exporter.isFolder(s)) {
          continue;
        }
        File f = new File(getPath(), s);
        if (f.isDirectory()) {
          if (f.getAbsolutePath().compareTo(mContext.getTempPath().getAbsolutePath()) != 0) {
            if (s.startsWith("20")) {
              data.add(0, s);
            } else {
              data.add(s);
            }
          }
        }
      }

      mItems.addAll(folders);
      mItems.addAll(data);
    }
    notifyDataSetChanged();
  }

  public boolean hasPosition() {
    if (mSelected.isEmpty()) {
      return false;
    }
    String key = (String)getItem(mSelected.get(0));
    File gpsFile = new File(new File(getPath(), key), "position.txt");
    return gpsFile.exists();
  }

  public void showPosition() {
    String key = (String)getItem(mSelected.get(0));
    File gpsFile = new File(new File(getPath(), key), "position.txt");
    try {
      Scanner sc = new Scanner(new FileInputStream(gpsFile.getAbsolutePath()));
      sc.useLocale(Locale.US);
      String lon = sc.next();
      String lat = sc.next();
      sc.close();

      Uri uri = Uri.parse("geo:" + lat + "," + lon + "");
      Intent intent = new Intent(android.content.Intent.ACTION_VIEW, uri);
      mContext.startActivity(intent);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void deleteModel() {
    AlertDialog.Builder deleteDlg = new AlertDialog.Builder(mContext);
    deleteDlg.setTitle(mContext.getString(R.string.delete));
    deleteDlg.setMessage(mContext.getString(R.string.continue_question));
    deleteDlg.setPositiveButton(mContext.getString(android.R.string.ok), (dialogInterface, i) -> {
      for (Integer index : mSelected) {
        String key = (String)getItem(index);
        AbstractActivity.deleteRecursive(new File(getPath(), key));
      }
      mContext.refreshUI();
    });
    deleteDlg.setNegativeButton(mContext.getString(android.R.string.cancel), null);

    AlertDialog d = deleteDlg.create();
    d.getWindow().setBackgroundDrawable(mContext.getDrawable(R.drawable.background_dialog));
    d.show();
  }

  public void shareModel() {
    String key = (String)getItem(mSelected.get(0));
    if (key.length() <= 4) {
      Toast.makeText(mContext, R.string.invalid_name, Toast.LENGTH_LONG).show();
    } else if ((key.endsWith(Exporter.EXT_DATASET)) || (key.endsWith(Exporter.EXT_PLY))) {
      mContext.showProgress();
      new Thread(() -> {
        final String zip = Exporter.compressModel(new File(getPath(), key));
        mContext.runOnUiThread(() -> {
          Intent intent = new Intent(Intent.ACTION_SEND);
          intent.setType("application/zip");
          intent.putExtra(Intent.EXTRA_STREAM, FileProvider.getUriForFile(mContext, BuildConfig.APPLICATION_ID + ".provider", new File(zip)));
          mContext.startActivity(Intent.createChooser(intent, mContext.getString(R.string.share_via)));
        });
      }).start();
    } else {
      AlertDialog.Builder dialog = new AlertDialog.Builder(mContext);
      dialog.setTitle(R.string.share_via);
      dialog.setItems(R.array.shares, (dialog1, which) -> {
        mContext.showProgress();
        new Thread(() -> {
          File file = new File(getPath(), key);
          final String zip = which == 1 ? AbstractActivity.getModel(file).getAbsolutePath() : Exporter.compressModel(file);
          mContext.runOnUiThread(() -> {
            Intent intent;
            switch (which) {
              case 0: //intent
                intent = new Intent(Intent.ACTION_SEND);
                intent.setType("application/zip");
                intent.putExtra(Intent.EXTRA_STREAM, FileProvider.getUriForFile(mContext, BuildConfig.APPLICATION_ID + ".provider", new File(zip)));
                mContext.startActivity(Intent.createChooser(intent, mContext.getString(R.string.share_via)));
                break;
              case 1: //online
                intent = new Intent(mContext, Uploader.class);
                intent.putExtra(AbstractActivity.FILE_KEY, zip);
                intent.putExtra(AbstractActivity.URL_KEY, "https://anyconv.com/mesh-converter/");
                mContext.startActivity(intent);
                break;
              case 2: //sketchfab
                intent = new Intent(mContext, OAuth.class);
                intent.putExtra(AbstractActivity.FILE_KEY, zip);
                mContext.startActivity(intent);
                break;
            }
            mContext.refreshUI();
          });
        }).start();
      });
      AlertDialog d = dialog.create();
      d.getWindow().setBackgroundDrawable(mContext.getDrawable(R.drawable.background_dialog));
      d.show();
    }
  }

  public void rename() {
    String key = (String)getItem(mSelected.get(0));
    new RenameDialog(mContext, getPath(), key);
  }

  public void setSelected(View selection, int index) {
    if (hasParent() && (index == 0)) {
      return;
    }
    if (mSelected.contains(index)) {
      mSelected.remove((Integer) index);
      selection.setVisibility(View.GONE);
    } else {
      mSelected.add(index);
      selection.setVisibility(View.VISIBLE);
    }

    mContext.setOptions(mSelected.size());
  }

  public String getSelected() {
    if (mSelected.isEmpty()) {
      return null;
    } else if (mSelected.size() == 1) {
      return (String)getItem(mSelected.get(0));
    } else {
      return mContext.getString(R.string.more_items) + " (" + mSelected.size() + ")";
    }
  }

  public boolean hasExtension() {
    if (mSelected.isEmpty()) {
      return false;
    }

    String key = (String)getItem(mSelected.get(0));
    for (String ext : Exporter.FILE_EXT) {
      if (key.endsWith(ext)) {
        return true;
      }
    }
    return false;
  }

  public void forwardTouch(MotionEvent event) {
    mGesture.onTouchEvent(event);
  }
}
