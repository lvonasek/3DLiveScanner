package com.lvonasek.arcore3dscanner.sketchfab;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.annotations.SerializedName;

import java.util.List;
import java.util.concurrent.TimeUnit;

import okhttp3.MultipartBody;
import okhttp3.OkHttpClient;
import retrofit2.Call;
import retrofit2.converter.gson.GsonConverterFactory;
import retrofit2.converter.scalars.ScalarsConverterFactory;
import retrofit2.http.Header;
import retrofit2.http.Multipart;
import retrofit2.http.POST;
import retrofit2.http.Part;

public class Retrofit {

    private static final String API_URL = "https://api.sketchfab.com/v3/";

    private static retrofit2.Retrofit retrofit;

    public interface APIInterface {

        @Multipart
        @POST("models")
        Call<String> uploadModel(@Header("Authorization") String auth,
                                 @Part MultipartBody.Part file,
                                 @Part MultipartBody.Part source,
                                 @Part MultipartBody.Part name,
                                 @Part MultipartBody.Part description,
                                 @Part MultipartBody.Part isInspectable,
                                 @Part MultipartBody.Part isPublished,
                                 @Part MultipartBody.Part license,
                                 @Part MultipartBody.Part tags,
                                 @Part MultipartBody.Part options);
    }

    public static class Options {
        @SerializedName("shading")
        public String shading;
    }

    public static retrofit2.Retrofit getRetrofitInstance() {
        OkHttpClient client = new OkHttpClient.Builder()
                .connectTimeout(60, TimeUnit.SECONDS)
                .readTimeout(60,TimeUnit.SECONDS).build();

        if (retrofit == null) {
            Gson gson = new GsonBuilder().setLenient().create();
            retrofit = new retrofit2.Retrofit.Builder()
                    .baseUrl(API_URL).client(client)
                    .addConverterFactory(ScalarsConverterFactory.create())
                    .addConverterFactory(GsonConverterFactory.create(gson))
                    .build();
        }
        return retrofit;
    }
}
