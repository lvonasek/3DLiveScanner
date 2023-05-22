package com.lvonasek.arcore3dscanner.main;

import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.os.BatteryManager;
import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;

import com.lvonasek.arcore3dscanner.ui.AbstractActivity;

public class Indicators implements Runnable {

    private ActivityManager mActivityManager;
    private ActivityManager.MemoryInfo mMemoryInfo;
    private AbstractActivity mMain;
    private String mOverrideMessage;
    private boolean mRunning;

    private LinearLayout mLayoutInfo;
    private TextView mInfoLeft;
    private TextView mInfoRight;
    private TextView mInfoLog;
    private View mBattery;

    public Indicators(AbstractActivity main) {
        mMain = main;
        mOverrideMessage = null;
        mRunning = true;

        mLayoutInfo = main.findViewById(com.lvonasek.arcore3dscanner.R.id.layout_info);
        mInfoLeft = main.findViewById(com.lvonasek.arcore3dscanner.R.id.info_left);
        mInfoRight = main.findViewById(com.lvonasek.arcore3dscanner.R.id.info_right);
        mInfoLog = main.findViewById(com.lvonasek.arcore3dscanner.R.id.infolog);
        mBattery = main.findViewById(com.lvonasek.arcore3dscanner.R.id.info_battery);
        mInfoLog = main.findViewById(com.lvonasek.arcore3dscanner.R.id.infolog);

        mActivityManager = (ActivityManager) main.getSystemService(Activity.ACTIVITY_SERVICE);
        mMemoryInfo = new ActivityManager.MemoryInfo();
        mLayoutInfo.setVisibility(View.VISIBLE);
        new Thread(this).start();
    }

    public void disable() {
        mLayoutInfo.setVisibility(View.GONE);
        mRunning = false;
    }

    public void setOverrideMessage(String message) {
        mOverrideMessage = message;
        updateText("");
    }

    public static int getBatteryPercentage(Context context) {
        IntentFilter iFilter = new IntentFilter(Intent.ACTION_BATTERY_CHANGED);
        Intent batteryStatus = context.registerReceiver(null, iFilter);
        int level = batteryStatus != null ? batteryStatus.getIntExtra(BatteryManager.EXTRA_LEVEL, -1) : -1;
        int scale = batteryStatus != null ? batteryStatus.getIntExtra(BatteryManager.EXTRA_SCALE, -1) : -1;
        float batteryPct = level / (float) scale;
        return (int) (batteryPct * 100);
    }


    @Override
    public void run()
    {
        while (mRunning) {
            try
            {
                Thread.sleep(1000);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            mMain.runOnUiThread(() -> {
                //memory info
                mActivityManager.getMemoryInfo(mMemoryInfo);
                long freeMBs = mMemoryInfo.availMem / 1048576L;
                mInfoLeft.setText(freeMBs + " MB");

                //warning
                if (freeMBs < 400)
                    mInfoLeft.setTextColor(Color.RED);
                else
                    mInfoLeft.setTextColor(Color.WHITE);

                //battery state
                int bat = getBatteryPercentage(mMain);
                mInfoRight.setText(bat + "%");
                int icon = com.lvonasek.arcore3dscanner.R.drawable.ic_battery_0;
                if (bat > 10)
                    icon = com.lvonasek.arcore3dscanner.R.drawable.ic_battery_20;
                if (bat > 30)
                    icon = com.lvonasek.arcore3dscanner.R.drawable.ic_battery_40;
                if (bat > 50)
                    icon = com.lvonasek.arcore3dscanner.R.drawable.ic_battery_60;
                if (bat > 70)
                    icon = com.lvonasek.arcore3dscanner.R.drawable.ic_battery_80;
                if (bat > 90)
                    icon = com.lvonasek.arcore3dscanner.R.drawable.ic_battery_100;
                mBattery.setBackgroundResource(icon);

                //warning
                if (bat < 15)
                    mInfoRight.setTextColor(Color.RED);
                else
                    mInfoRight.setTextColor(Color.WHITE);

                //update info about AR
                updateText(JNI.getEvent(mMain.getResources()));
            });
        }
    }

    private void updateText(String text) {
        if (mOverrideMessage != null) {
            text = mOverrideMessage;
        }
        mInfoLog.setVisibility(text.length() > 0 ? View.VISIBLE : View.GONE);
        mInfoLog.setText(text);
    }
}