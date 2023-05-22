package com.lvonasek.arcore3dscanner.ui;

import android.content.Context;
import android.content.Intent;
import android.graphics.Color;

import java.io.InputStream;

public class Initializator extends AbstractActivity
{
  private static boolean closeOnResume = false;
  private static boolean first;
  private static Intent lastIntent;

  public Initializator()
  {
    super();
    first = true;
  }

  public static InputStream getFile(Context context) {
    try {
      InputStream is = context.getContentResolver().openInputStream(lastIntent.getData());
      lastIntent = null;
      return is;
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }

  @Override
  public int getNavigationBarColor() {
    return getStatusBarColor();
  }

  @Override
  public int getStatusBarColor() {
    return Color.BLACK;
  }

  public static boolean hasFileIntent() {
    return (lastIntent != null) && (lastIntent.getData() != null);
  }

  @Override
  protected void onResume()
  {
    super.onResume();

    lastIntent = getIntent();
    if (closeOnResume && !hasFileIntent()) {
      closeOnResume = false;
      if (Service.getRunning(this) > Service.SERVICE_NOT_RUNNING) {
        moveTaskToBack(true);
      } else {
        finish();
        System.exit(0);
      }
    } else {
      closeOnResume = true;
      Intent intent = new Intent(this, FileManager.class);
      intent.setFlags(Intent.FLAG_ACTIVITY_CLEAR_TASK);
      startActivity(intent);
    }
  }

  public static boolean isFirst()
  {
    boolean output = first;
    first = false;
    return output;
  }
}
