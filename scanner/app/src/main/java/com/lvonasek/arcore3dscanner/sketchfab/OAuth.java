package com.lvonasek.arcore3dscanner.sketchfab;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.net.Uri;
import android.os.Bundle;

import com.lvonasek.arcore3dscanner.R;
import com.lvonasek.arcore3dscanner.ui.AbstractActivity;

public class OAuth extends AbstractActivity
{
  private static final String CLIENT_ID = "EBLAsbUo972Ki4GaMWryOsHVhGrt2Vhke98vLPxS";
  private static final String OAUTH_URL = "https://sketchfab.com/oauth2/authorize";
  private static final String REDIRECT  = "https://lvonasek.github.io/redirect.html";
  private static final String TOKEN_URL = "https://sketchfab.com/oauth2/token";
  private static final int PERMISSIONS_CODE = 1989;

  private static String token = null;
  private static String mExtra;

  public static String getToken()
  {
    return token;
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_upload);
    String file = getIntent().getStringExtra(FILE_KEY);
    if (file != null) {
      mExtra = getIntent().getStringExtra(FILE_KEY);
      setupPermissions();
    } else {
      String data = getIntent().getData().toString();
      if (data.contains("access_token")) {
        token = data.substring(data.indexOf("access_token"));
        token = token.substring(token.indexOf("=") + 1);
        token = token.substring(0, token.indexOf("&"));

        Intent intent = new Intent(this, Uploader.class);
        intent.putExtra(AbstractActivity.FILE_KEY, mExtra);
        startActivity(intent);
      }
      finish();
    }
  }

  protected void setupPermissions() {
    String[] permissions = {
            Manifest.permission.INTERNET
    };
    boolean ok = true;
    for (String s : permissions)
      if (checkSelfPermission(s) != PackageManager.PERMISSION_GRANTED)
        ok = false;

    if (!ok)
      requestPermissions(permissions, PERMISSIONS_CODE);
    else
      onRequestPermissionsResult(PERMISSIONS_CODE, null, new int[]{PackageManager.PERMISSION_GRANTED});
  }


  @Override
  public synchronized void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults)
  {
    if (requestCode == PERMISSIONS_CODE) {
      if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
        //TODO:https://github.com/openid/AppAuth-Android
        String url = OAUTH_URL + "/?response_type=token&client_id=" + CLIENT_ID + "&redirect_uri=" + REDIRECT;
        startActivity(new Intent(Intent.ACTION_VIEW, Uri.parse(url)));
      }
      finish();
    }
  }

  @Override
  public int getNavigationBarColor() {
    return Color.WHITE;
  }

  @Override
  public int getStatusBarColor() {
    return Color.WHITE;
  }
}
