package com.lvonasek.record;

import android.app.Activity;
import android.content.ContentResolver;
import android.content.ContentValues;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.media.AudioManager;
import android.media.MediaActionSound;
import android.media.MediaCodec;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import android.media.MediaMuxer;
import android.media.MediaRecorder;
import android.media.MediaScannerConnection;
import android.net.Uri;
import android.os.Environment;
import android.provider.MediaStore;
import android.util.Log;

import com.lvonasek.gles.GLESSurfaceView;

import org.jcodec.api.SequenceEncoder;
import org.jcodec.common.Codec;
import org.jcodec.common.Format;
import org.jcodec.common.io.NIOUtils;
import org.jcodec.common.io.SeekableByteChannel;
import org.jcodec.common.model.Picture;
import org.jcodec.common.model.Rational;
import org.jcodec.scale.BitmapUtil;

import java.io.File;
import java.io.FileDescriptor;
import java.io.FileOutputStream;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import javax.microedition.khronos.opengles.GL10;

public class Recorder {

    //audio objects
    private static File mAudioFile;
    private static MediaRecorder mAudioEncoder;

    //video objects
    private static File mVideoFile;
    private static SequenceEncoder mVideoEncoder;
    private static int mVideoFrames;
    private static SeekableByteChannel mVideoOut;
    private static long mVideoTimestamp;

    //video dimensions
    private static int mWidth;
    private static int mHeight;
    private static int mScale;
    private static int mOrientation;

    //video capturing
    private static int[] mBitmapBuffer;
    private static int[] mBitmapSource;
    private static IntBuffer mIntBuffer;
    private static final Object mLock = new Object();
    private static boolean mRecording;
    private static int mThreads;

    //settings
    private static File mCustomRoot = null;
    private static int mVideoDownscale = 640;
    private static int mVideoFPS = 15;

    public static void capturePhoto(GL10 gl, GLESSurfaceView view) {
        int w = view.getWidth();
        int h = view.getHeight();
        Bitmap bitmap = createBitmapFromGLSurface(0, 0, w, h, gl, 1);
        if (bitmap != null) {
            Context context = view.getContext();
            playShooterSound(context, MediaActionSound.SHUTTER_CLICK);
            try {
                FileDescriptor file = context.getContentResolver().openFileDescriptor(getFile(context, false), "rw").getFileDescriptor();
                FileOutputStream out = new FileOutputStream(file);
                bitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
                out.flush();
                out.close();

                MediaScannerConnection.scanFile(context,
                        new String[] { file.toString() }, null,
                        (path, uri) -> {
                            Log.i("ExternalStorage", "Scanned " + path + ":");
                            Log.i("ExternalStorage", "-> uri=" + uri);
                        });
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static void captureVideoFrame(GL10 gl, GLESSurfaceView view, boolean addTimestamp, int frameSkip, boolean multithread) {
        if (!isVideoRecording()) {
            return;
        }

        int count = 0;
        if (frameSkip <= 0) {
            while (true) {
                long expectedTimeStamp = mVideoTimestamp + mVideoFrames * 1000 / mVideoFPS;
                boolean ok = expectedTimeStamp <= System.currentTimeMillis();
                if (ok) {
                    count++;
                    mVideoFrames++;
                } else {
                    break;
                }
            }
        } else {
            count = mVideoFrames % frameSkip == 0 ? 1 : 0;
            mVideoFrames++;
        }
        if (count > 0) {
            int w = view.getWidth();
            int h = view.getHeight();
            int s = Math.max(w, h) / mVideoDownscale + 1;
            createCaches(w, h, s);
            SimpleDateFormat formatterDate = new SimpleDateFormat("dd.MM.yyyy", Locale.US);
            SimpleDateFormat formatterTime = new SimpleDateFormat("HH:mm", Locale.US);
            String date = formatterDate.format(new Date(System.currentTimeMillis()));
            String time = formatterTime.format(new Date(System.currentTimeMillis()));
            gl.glReadPixels(0, 0, w, h, GL10.GL_RGBA, GL10.GL_UNSIGNED_BYTE, mIntBuffer);
            mThreads++;
            if (multithread) {
                int finalCount = count;
                Runnable r = () -> {
                    captureFrame(w, h, s, date, time, finalCount, addTimestamp);
                    mThreads--;
                };
                if (frameSkip <= 0) {
                    new Thread(r).start();
                } else {
                    r.run();
                }
            } else {
                captureFrame(w, h, s, date, time, count, addTimestamp);
                mThreads--;
            }
        }
    }

    public static boolean isVideoRecording() {
        return mRecording && mVideoEncoder != null;
    }

    public static void startCapturingVideo(Activity context, boolean recordAudio) {

        //lock screen orientation
        mOrientation = context.getRequestedOrientation();
        context.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LOCKED);
        playShooterSound(context, MediaActionSound.START_VIDEO_RECORDING);

        //prepare audio
        if (recordAudio) {
            mAudioFile = new File(getRootPath(), "audio_render.3gp");
            mAudioEncoder = new MediaRecorder();
            mAudioEncoder.setAudioSource(MediaRecorder.AudioSource.MIC);
            mAudioEncoder.setOutputFormat(MediaRecorder.OutputFormat.THREE_GPP);
            mAudioEncoder.setAudioEncoder(MediaRecorder.AudioEncoder.AMR_NB);
            mAudioEncoder.setAudioEncodingBitRate(128000);
            mAudioEncoder.setAudioSamplingRate(44100);
            mAudioEncoder.setOutputFile(mAudioFile.getAbsolutePath());
            try {
                mAudioEncoder.prepare();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        //prepare video
        try {
            Rational fps = Rational.R(mVideoFPS, 1);
            mVideoFile = new File(getRootPath(), "video_render.mp4");
            mVideoOut = NIOUtils.writableFileChannel(mVideoFile.getAbsolutePath());
            mVideoEncoder = new SequenceEncoder(mVideoOut, fps, Format.MOV, Codec.H264, null);
            mVideoFrames = 0;
            mVideoTimestamp = System.currentTimeMillis();
        } catch (Exception e) {
            e.printStackTrace();
        }

        //start
        if (recordAudio) {
            try {
                mAudioEncoder.start();
                mVideoTimestamp = System.currentTimeMillis();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        mRecording = true;
    }

    public static void stopCapturingVideo(Activity context, boolean saveIntoGallery) {

        //cancel recording
        mRecording = false;

        //stop audio
        if (mAudioEncoder != null) {
            mAudioEncoder.stop();
            mAudioEncoder.release();
            mAudioEncoder = null;
        }

        //stop video
        try {
            while (mThreads > 0) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            mVideoEncoder.finish();
            mVideoOut.close();
            mVideoEncoder = null;
            mVideoOut = null;
        } catch (Exception e) {
            e.printStackTrace();
        }

        //mix files
        if (saveIntoGallery) {
            mixTracks(context);
            mVideoFile.delete();
            if (mAudioFile != null) {
                mAudioFile.delete();
            }
            mAudioFile = null;
            mVideoFile = null;
        }

        playShooterSound(context, MediaActionSound.STOP_VIDEO_RECORDING);
        context.setRequestedOrientation(mOrientation);
    }

    public static File getVideoFile() {
        return mVideoFile;
    }

    public static int getVideoFPS() {
        return mVideoFPS;
    }

    public static void setCustomRoot(File file) {
        mCustomRoot = file;
    }

    public static void setVideoDownscale(int downscale) {
        mVideoDownscale = downscale;
    }

    public static void setVideoFPS(int fps) {
        mVideoFPS = fps;
    }

    private static Bitmap createBitmapFromBitmapBuffer(int w, int h, int[] bitmapBuffer, int s) {
        int ws = w / s;
        int hs = h / s;
        if (ws % 2 == 1) ws--;
        if (hs % 2 == 1) hs--;

        try {
            int offset1, offset2, texturePixel;
            for (int i = 0; i < hs; i++) {
                offset1 = i * s * w;
                offset2 = (hs - i - 1) * ws;
                for (int j = 0; j < ws; j++) {
                    texturePixel = bitmapBuffer[offset1 + j * s];
                    mBitmapSource[offset2 + j] = (texturePixel & 0xff00ff00) | (texturePixel << 16) & 0x00ff0000 | (texturePixel >> 16) & 0xff;
                }
            }
        } catch (Exception e) {
            return null;
        }
        return Bitmap.createBitmap(mBitmapSource, ws, hs, Bitmap.Config.ARGB_8888);
    }

    private static Bitmap createBitmapFromGLSurface(int x, int y, int w, int h, GL10 gl, int s) {
        createCaches(w, h, s);
        gl.glReadPixels(x, y, w, h, GL10.GL_RGBA, GL10.GL_UNSIGNED_BYTE, mIntBuffer);
        return createBitmapFromBitmapBuffer(w, h, mBitmapBuffer, s);
    }

    private static void createCaches(int w, int h, int s) {
        if ((w != mWidth) || (h != mHeight) || (s != mScale)) {
            int ws = w / s;
            int hs = h / s;
            if (ws % 2 == 1) ws--;
            if (hs % 2 == 1) hs--;
            mBitmapSource = new int[ws * hs];
            mBitmapBuffer = new int[w * h];
            mIntBuffer = IntBuffer.wrap(mBitmapBuffer);
            mWidth = w;
            mHeight = h;
            mScale = s;
        }
        mIntBuffer.position(0);
    }

    private static Uri getFile(Context context, boolean video) {
        SimpleDateFormat formatter = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);
        Date date = new Date(System.currentTimeMillis());
        String pkg = context.getPackageName();
        String type = video ? "Video_" : "Photo_";
        String extension = video ? ".mp4" : ".jpg";
        String name = type + formatter.format(date) + "_" + pkg + extension;

        ContentResolver resolver = context.getContentResolver();
        ContentValues contentValues = new ContentValues();
        contentValues.put(MediaStore.MediaColumns.DISPLAY_NAME, name);
        contentValues.put(MediaStore.MediaColumns.MIME_TYPE, video ? "video/mp4": "image/jpeg");
        contentValues.put(MediaStore.MediaColumns.RELATIVE_PATH, Environment.DIRECTORY_DCIM);

        return resolver.insert(video ? MediaStore.Video.Media.EXTERNAL_CONTENT_URI : MediaStore.Images.Media.EXTERNAL_CONTENT_URI, contentValues);
    }

    private static void mixTracks(Context context) {
        try {
            FileDescriptor file = context.getContentResolver().openFileDescriptor(getFile(context, true), "rw").getFileDescriptor();

            MediaExtractor videoExtractor = new MediaExtractor();
            videoExtractor.setDataSource(mVideoFile.getAbsolutePath());

            MediaExtractor audioExtractor = new MediaExtractor();
            if (mAudioFile != null) {
                audioExtractor.setDataSource(mAudioFile.getAbsolutePath());
            }

            MediaMuxer muxer = new MediaMuxer(file, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4);

            videoExtractor.selectTrack(0);
            MediaFormat videoFormat = videoExtractor.getTrackFormat(0);
            int videoTrack = muxer.addTrack(videoFormat);

            int audioTrack = 0;
            if (mAudioFile != null) {
                audioExtractor.selectTrack(0);
                MediaFormat audioFormat = audioExtractor.getTrackFormat(0);
                audioTrack = muxer.addTrack(audioFormat);
            }

            boolean sawEOS = false;
            int offset = 100;
            int sampleSize = 256 * 1024;
            ByteBuffer videoBuf = ByteBuffer.allocate(sampleSize);
            ByteBuffer audioBuf = ByteBuffer.allocate(sampleSize);
            MediaCodec.BufferInfo videoBufferInfo = new MediaCodec.BufferInfo();
            MediaCodec.BufferInfo audioBufferInfo = new MediaCodec.BufferInfo();


            videoExtractor.seekTo(0, MediaExtractor.SEEK_TO_CLOSEST_SYNC);
            if (mAudioFile != null) {
                audioExtractor.seekTo(0, MediaExtractor.SEEK_TO_CLOSEST_SYNC);
            }

            muxer.start();

            while (!sawEOS)
            {
                videoBufferInfo.offset = offset;
                videoBufferInfo.size = videoExtractor.readSampleData(videoBuf, offset);


                if (videoBufferInfo.size < 0 || audioBufferInfo.size < 0)
                {
                    sawEOS = true;
                    videoBufferInfo.size = 0;
                }
                else
                {
                    videoBufferInfo.presentationTimeUs = videoExtractor.getSampleTime();
                    videoBufferInfo.flags = videoExtractor.getSampleFlags();
                    muxer.writeSampleData(videoTrack, videoBuf, videoBufferInfo);
                    videoExtractor.advance();
                }
            }

            if (mAudioFile != null) {

                boolean sawEOS2 = false;
                while (!sawEOS2)
                {
                    audioBufferInfo.offset = offset;
                    audioBufferInfo.size = audioExtractor.readSampleData(audioBuf, offset);

                    if (videoBufferInfo.size < 0 || audioBufferInfo.size < 0)
                    {
                        sawEOS2 = true;
                        audioBufferInfo.size = 0;
                    }
                    else
                    {
                        audioBufferInfo.presentationTimeUs = audioExtractor.getSampleTime();
                        audioBufferInfo.flags = audioExtractor.getSampleFlags();
                        muxer.writeSampleData(audioTrack, audioBuf, audioBufferInfo);
                        audioExtractor.advance();
                    }
                }
            }

            muxer.stop();
            muxer.release();

            MediaScannerConnection.scanFile(context,
                    new String[] { file.toString() }, null,
                    (path, uri) -> {
                        Log.i("ExternalStorage", "Scanned " + path + ":");
                        Log.i("ExternalStorage", "-> uri=" + uri);
                    });

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static void playShooterSound(Context context, int sample) {
        AudioManager audio = (AudioManager) context.getSystemService(Context.AUDIO_SERVICE);
        switch( audio.getRingerMode() ){
            case AudioManager.RINGER_MODE_NORMAL:
                MediaActionSound sound = new MediaActionSound();
                sound.play(sample);
                break;
            case AudioManager.RINGER_MODE_SILENT:
                break;
            case AudioManager.RINGER_MODE_VIBRATE:
                break;
        }
    }

    private static void captureFrame(int w, int h, int s, String date, String time, int count, boolean addTimestamp) {
        synchronized (mLock) {
            if (isVideoRecording()) {
                Bitmap bitmap = createBitmapFromBitmapBuffer(w, h, mBitmapBuffer, s);
                if (bitmap != null) {
                    try {
                        Picture picture;
                        if (addTimestamp) {
                            Paint paint = new Paint();
                            paint.setAntiAlias(true);
                            paint.setColor(Color.WHITE);
                            paint.setTextSize(16);
                            Bitmap mutableBitmap = bitmap.copy(Bitmap.Config.ARGB_8888, true);
                            Canvas canvas = new Canvas(mutableBitmap);
                            canvas.drawText(date, 16, 16, paint);
                            canvas.drawText(time, 16, 16 + paint.getTextSize(), paint);
                            picture = BitmapUtil.fromBitmap(mutableBitmap);
                        } else {
                            picture = BitmapUtil.fromBitmap(bitmap);
                        }
                        for (int i = 0; i < count; i++) {
                            mVideoEncoder.encodeNativeFrame(picture);
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    private static File getRootPath() {
        if (mCustomRoot != null) {
            return mCustomRoot;
        }
        return Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
    }
}