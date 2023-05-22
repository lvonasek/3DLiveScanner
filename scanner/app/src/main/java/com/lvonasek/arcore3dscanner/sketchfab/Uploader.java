package com.lvonasek.arcore3dscanner.sketchfab;

import android.app.AlertDialog;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;

import com.google.gson.Gson;
import com.lvonasek.arcore3dscanner.R;
import com.lvonasek.arcore3dscanner.ui.AbstractActivity;
import com.lvonasek.arcore3dscanner.ui.Service;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import okhttp3.MediaType;
import okhttp3.MultipartBody;
import okhttp3.RequestBody;
import retrofit2.Call;
import retrofit2.Response;

public class Uploader extends AbstractActivity implements OnClickListener
{
  private EditText mFilename;
  private EditText mDescription;
  private EditText mTags;
  private Button mButtonOk;
  private String mFile;
  private String mModelUri;

  @Override
  protected void onCreate(Bundle savedInstanceState)
  {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_sketchfab);

    mFilename = findViewById(R.id.editText_filename);
    mDescription = findViewById(R.id.editText_description);
    mTags = findViewById(R.id.editText_tags);
    mButtonOk = findViewById(R.id.button_ok);

    mFile = getIntent().getStringExtra(FILE_KEY);
    mButtonOk.setEnabled(false);
    mButtonOk.setOnClickListener(this);

    mFilename.addTextChangedListener(new TextWatcher()
    {

      @Override
      public void afterTextChanged(Editable s)
      {
      }

      @Override
      public void beforeTextChanged(CharSequence s, int start, int count, int after)
      {
      }

      @Override
      public void onTextChanged(CharSequence s, int start, int before, int count)
      {
        mButtonOk.setEnabled(s.length() != 0);
      }
    });

    mFilename.setSelectAllOnFocus(true);
    mFilename.requestFocus();
  }

  @Override
  public void onClick(View v)
  {
    if (v.getId() == R.id.button_ok)
      publish();
  }

  private static String getDeviceName()
  {
    String output = "";
    output += Build.MANUFACTURER + " ";
    output += Build.MODEL;
    return output.replaceAll(" ", "_");
  }

  private ArrayList<String> getTags()
  {
    ArrayList<String> output = new ArrayList<>();
    output.add("3dlivescanner");
    output.add(" scanned_by_" + getDeviceName());
    for (String s : mTags.getText().toString().split("\\\\s+")) {
      output.add(" " + s);
    }
    return output;
  }

  private void publish()
  {
    Retrofit.Options cfg = new Retrofit.Options();
    cfg.shading = "shadeless";
    File f = new File(mFile);

    // Continue?
    AlertDialog.Builder builder = new AlertDialog.Builder(Uploader.this);
    builder.setTitle(R.string.sketchfab_upload_ready);

    final int fileSizeMB = (int) f.length() / (1024 * 1024);
    final int fileSizeKB = (int) f.length() / (1024);
    if (fileSizeMB == 0)
      builder.setMessage(getString(R.string.upload_size_is) + " " + fileSizeKB + " KB. " + getString(R.string.continue_question));
    else
    {
      String warning = fileSizeMB >= 100 ? getString(R.string.sketchfab_pro) : "";
      builder.setMessage(getString(R.string.upload_size_is) + " " + fileSizeMB + " MB. " + getString(R.string.continue_question) + warning);
    }

    builder.setPositiveButton(android.R.string.yes, (dialog, which) -> Service.process(getString(R.string.sketchfab_uploading), Service.SERVICE_SKETCHFAB,
            Uploader.this, () -> {
              finish();

              File uploadFile = new File(mFile);

              Retrofit.APIInterface service = Retrofit.getRetrofitInstance().create(Retrofit.APIInterface.class);

              String header = "Bearer " + OAuth.getToken();
              RequestBody requestFile = RequestBody.create(MediaType.parse("application/zip"), uploadFile);
              MultipartBody.Part file = MultipartBody.Part.createFormData("modelFile", uploadFile.getName(), requestFile);
              MultipartBody.Part source = MultipartBody.Part.createFormData("source", "3d-live-scanner");
              MultipartBody.Part name = MultipartBody.Part.createFormData("name", mFilename.getText().toString());
              MultipartBody.Part description = MultipartBody.Part.createFormData("description", mDescription.getText().toString());
              MultipartBody.Part isInspectable = MultipartBody.Part.createFormData("isInspectable", "true");
              MultipartBody.Part isPublished = MultipartBody.Part.createFormData("isPublished", "false");
              MultipartBody.Part license = MultipartBody.Part.createFormData("license", "by-sa");
              MultipartBody.Part tags = MultipartBody.Part.createFormData("tags", new Gson().toJson(getTags()));
              MultipartBody.Part options = MultipartBody.Part.createFormData("options", new Gson().toJson(cfg));

              Call<String> call = service.uploadModel(header, source, file, name, description, isInspectable, isPublished, license, tags, options);
              try {
                Response<String> response = call.execute();
                if (response.code() != 201) {
                  Exception e = new Exception(response.errorBody().string());
                  e.printStackTrace();
                  Service.forceState(this, null, Service.SERVICE_SKETCHFAB);
                } else {
                  try {
                    mModelUri = (String)new JSONObject(response.body()).get("uid");
                    Service.finish("https://sketchfab.com/models/" + mModelUri);
                  } catch (JSONException e) {
                    e.printStackTrace();
                    Service.forceState(this, null, Service.SERVICE_SKETCHFAB);
                  }
                }

                //Service.finish("https://sketchfab.com/models/" + mModelUri);
              } catch (IOException e) {
                e.printStackTrace();
                Service.forceState(this, null, Service.SERVICE_SKETCHFAB);
              }
            }));
    builder.setNegativeButton(android.R.string.no, (dialog, which) -> {});
    builder.show();
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