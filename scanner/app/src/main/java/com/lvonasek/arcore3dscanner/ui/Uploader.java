package com.lvonasek.arcore3dscanner.ui;

import android.Manifest;
import android.app.DownloadManager;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.webkit.CookieManager;
import android.webkit.DownloadListener;
import android.webkit.URLUtil;
import android.webkit.ValueCallback;
import android.webkit.WebChromeClient;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

import androidx.core.content.FileProvider;

import com.lvonasek.arcore3dscanner.R;
import java.io.File;


public class Uploader extends AbstractActivity {

  private String mFile;
  private Uri mUri;
  private String mUrl;
  private WebView mWebView;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_upload);

    mFile = getIntent().getStringExtra(FILE_KEY);
    mUrl = getIntent().getStringExtra(URL_KEY);
    mWebView = findViewById(R.id.webview);
    WebSettings webSettings = mWebView.getSettings();
    webSettings.setUserAgentString("Mozilla/5.0 Google");
    webSettings.setJavaScriptEnabled(true);
    webSettings.setUseWideViewPort(true);
    webSettings.setLoadWithOverviewMode(true);
    webSettings.setAllowFileAccess(true);
    webSettings.setJavaScriptCanOpenWindowsAutomatically(true);
    webSettings.setBuiltInZoomControls(true);
    webSettings.setPluginState(WebSettings.PluginState.ON);
    webSettings.setSupportZoom(true);
    webSettings.setAllowContentAccess(true);

    mWebView.setWebViewClient(new WebViewClient());
    if (mFile != null) {
      mWebView.setWebChromeClient(new WebChromeClient()
      {
        public boolean onShowFileChooser(WebView webView, ValueCallback<Uri[]> callback, WebChromeClient.FileChooserParams params){
          callback.onReceiveValue(new Uri[]{mUri});
          return true;
        }
      });
    }
    mWebView.setDownloadListener((url, userAgent, contentDisposition, mimeType, contentLength) -> {
      DownloadManager.Request request = new DownloadManager.Request(Uri.parse(url));
      request.setMimeType(mimeType);
      String filename = contentDisposition.substring(contentDisposition.indexOf('\"') + 1);
      filename = filename.substring(0, filename.indexOf('\"'));
      String cookies = CookieManager.getInstance().getCookie(url);
      request.addRequestHeader("cookie", cookies);
      request.addRequestHeader("User-Agent", userAgent);
      request.setTitle(filename);
      request.allowScanningByMediaScanner();
      request.setNotificationVisibility(DownloadManager.Request.VISIBILITY_VISIBLE_NOTIFY_COMPLETED);
      request.setDestinationInExternalPublicDir(Environment.DIRECTORY_DOWNLOADS, filename);
      DownloadManager dm = (DownloadManager) getSystemService(DOWNLOAD_SERVICE);
      dm.enqueue(request);
    });
    setupPermissions();
  }


  protected void setupPermissions() {
    String[] permissions = {
            Manifest.permission.INTERNET
    };

    onPermissionFail = () -> finish();
    onPermissionSuccess = () -> {
      mUri = FileProvider.getUriForFile(Uploader.this, getApplicationContext().getPackageName() + ".provider", new File(mFile));
      mWebView.loadUrl(mUrl);
    };
    askForPermissions(permissions);
  }

  @Override
  public int getNavigationBarColor() {
    return getStatusBarColor();
  }

  @Override
  public int getStatusBarColor() {
    return Color.WHITE;
  }
}
