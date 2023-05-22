package com.lvonasek.utils;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class Compass implements SensorEventListener
{
    private static float        yaw;

    private SensorManager       mSensorManager;
    private Sensor              mSensorCompass;

    public Compass(Context c)
    {
        mSensorManager = (SensorManager)c.getSystemService(Context.SENSOR_SERVICE);
        mSensorCompass = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    }

    public void onPause()
    {
        if (mSensorCompass != null)
            mSensorManager.unregisterListener(this, mSensorCompass);
    }

    public void onResume()
    {
        if (mSensorCompass != null)
            mSensorManager.registerListener(this, mSensorCompass, SensorManager.SENSOR_DELAY_GAME);
    }

    @Override
    public void onSensorChanged(SensorEvent event)
    {
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            yaw = (float) (360.0f * Math.atan2(event.values[0], event.values[2]) / Math.PI);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int acc)
    {
    }

    public static float getValue()
    {
        return yaw;
    }
}
