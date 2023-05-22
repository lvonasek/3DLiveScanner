package com.lvonasek.arcore3dscanner.ui;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.IBinder;
import android.preference.PreferenceManager;
import android.util.Log;

import com.lvonasek.arcore3dscanner.main.JNI;

public class Service extends android.app.Service
{
  public static final String SERVICE_LINK = "service_link";
  public static final String SERVICE_RUNNING = "service_running";

  public static final int SERVICE_NOT_RUNNING = 0;
  public static final int SERVICE_POSTPROCESS = 1;
  public static final int SERVICE_SAVE = 2;
  public static final int SERVICE_SKETCHFAB = 3;
  public static final int SERVICE_PHOTOGRAMMETRY = 4;

  private static Runnable action;
  private static String message;
  private static String messageNotification;
  private static AbstractActivity parent;
  private static boolean running;
  private static Service service;

  @Override
  public synchronized void onCreate() {
    super.onCreate();
    service = this;
    message = "";
    if (parent == null)
      return;
    if ((getRunning(parent) == SERVICE_POSTPROCESS) || (getRunning(parent) == SERVICE_SAVE)) {
      running = true;
      new Thread(() -> {
        while(running) {
          setMessage(JNI.getEvent(Service.this.getResources()));
          try
          {
            Thread.sleep(1000);
          } catch (Exception e)
          {
            e.printStackTrace();
          }
        }
        message = "";
      }).start();
    }
    new Thread(() -> action.run()).start();
  }

  @Override
  public int onStartCommand(Intent intent, int flags, int startId)
  {
    return START_STICKY;
  }

  @Override
  public IBinder onBind(Intent intent)
  {
    return null;
  }

  public static synchronized void finish(String link)
  {
    running = false;
    service.stopService(new Intent(parent, Service.class));
    SharedPreferences.Editor e = PreferenceManager.getDefaultSharedPreferences(parent).edit();
    e.putInt(SERVICE_RUNNING, -Math.abs(getRunning(parent)));
    e.putString(SERVICE_LINK, link);
    e.commit();
    System.exit(0);
  }

  public static synchronized void forceState(AbstractActivity activity, String link, int state)
  {
    running = false;
    SharedPreferences.Editor e = PreferenceManager.getDefaultSharedPreferences(activity).edit();
    e.putInt(SERVICE_RUNNING, -Math.abs(state));
    e.putString(SERVICE_LINK, link);
    e.commit();
    System.exit(0);
  }

  public static synchronized void interrupt() {
    messageNotification = null;
    message = null;
  }

  public static synchronized void process(String message, int serviceId, AbstractActivity activity, Runnable runnable)
  {
    action = runnable;
    parent = activity;
    messageNotification = message;

    SharedPreferences.Editor e = PreferenceManager.getDefaultSharedPreferences(activity).edit();
    e.putInt(SERVICE_RUNNING, serviceId);
    e.putString(SERVICE_LINK, "");
    e.commit();
    activity.runOnUiThread(() -> activity.startService(new Intent(activity, Service.class)));
  }

  public static synchronized String getLink(Context context)
  {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
    return pref.getString(SERVICE_LINK, "");
  }

  public static synchronized String getMessage()
  {
    if (messageNotification == null)
      return null;
    if (message == null)
      return null;
    return messageNotification + "\n" + message;
  }

  public static synchronized int getRunning(Context context)
  {
    SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
    return pref.getInt(SERVICE_RUNNING, SERVICE_NOT_RUNNING);
  }

  private static synchronized void setMessage(String msg)
  {
    message = msg;
  }

  public static synchronized void setMessageNotification(String msg)
  {
    messageNotification = msg;
  }

  public static synchronized void reset(Context context)
  {
    try
    {
      service.stopService(new Intent(parent, Service.class));
    } catch(Exception e)
    {
      e.printStackTrace();
    }
    SharedPreferences.Editor e = PreferenceManager.getDefaultSharedPreferences(context).edit();
    e.putInt(SERVICE_RUNNING, SERVICE_NOT_RUNNING);
    e.putString(SERVICE_LINK, "");
    e.commit();
  }
}