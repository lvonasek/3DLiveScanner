package com.lvonasek.arcore3dscanner.main;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Align;
import android.graphics.Paint.Style;
import android.util.AttributeSet;
import android.view.View;

import java.util.Locale;

public class DistanceMeasuring extends View
{
  private Paint                mPaint;
  private int                  mPointerX1    = 0;
  private int                  mPointerY1    = 0;
  private int                  mPointerX2    = 0;
  private int                  mPointerY2    = 0;
  private boolean              mHaveCoords   = false;
  private int                  mPointerCount = 0;

  public DistanceMeasuring(Context c, AttributeSet attrs)
  {
    super(c, attrs);
    if(isInEditMode())
      return;
    mPaint = new Paint();
    mPaint.setTextAlign(Align.CENTER);
  }

  @Override
  public void onDraw(Canvas c)
  {
    if(isInEditMode())
      return;
    int highlight = getResources().getColor(android.R.color.holo_blue_bright);

    // distance measuring by two fingers
    if (mHaveCoords && (mPointerCount < 2)) {
      float d = getResources().getDisplayMetrics().density;
      mPaint.setShadowLayer(5, 0, 0, Color.GRAY);
      mPaint.setStyle(Style.FILL_AND_STROKE);
      mPaint.setStrokeWidth(d);
      mPaint.setTextSize(d * 20);
      mPaint.setColor(Color.GRAY);
      for (int i = -1; i <= 1; i += 2)
        for (int j = -1; j <= 1; j += 2)
          drawArrow(c, mPointerX1 + i, mPointerY1 + j, mPointerX2 + i, mPointerY2 + j, mPaint);
      mPaint.setColor(highlight);
      drawArrow(c, mPointerX1, mPointerY1, mPointerX2, mPointerY2, mPaint);

      float value = JNI.getDistance(mPointerX1, mPointerY1, mPointerX2, mPointerY2);
      if ((value < 0.01f) || (value > 5000)) {
        reset();
        return;
      }
      String text = "";
      if (value > 1)
        text = String.format(Locale.US, "%.2f", value) + "m";
      else
        text = String.format(Locale.US, "%.2f", value * 100.0f) + "cm";
      mPaint.setStyle(Style.STROKE);
      mPaint.setColor(Color.GRAY);
      c.drawText(text, (mPointerX1 + mPointerX2) / 2, (mPointerY1 + mPointerY2) / 2, mPaint);
      mPaint.setStyle(Style.FILL);
      mPaint.setColor(highlight);
      c.drawText(text, (mPointerX1 + mPointerX2) / 2, (mPointerY1 + mPointerY2) / 2, mPaint);
    }
  }

  public void drawArrow(Canvas c, int x1, int y1, int x2, int y2, Paint mPaint)
  {
    c.drawLine(x1, y1, x2, y2, mPaint);
    double ang = Math.atan2(x1 - x2, y1 - y2);
    double a1 = Math.PI / 8.0f + Math.PI;
    double a2 = Math.PI / 8.0f;
    float d = getResources().getDisplayMetrics().density * 15;
    c.drawLine(x1, y1, x1 + (int)(Math.sin(ang + a1) * d), y1 + (int)(Math.cos(ang + a1) * d), mPaint);
    c.drawLine(x1, y1, x1 + (int)(Math.sin(ang - a1) * d), y1 + (int)(Math.cos(ang - a1) * d), mPaint);
    c.drawLine(x2, y2, x2 + (int)(Math.sin(ang + a2) * d), y2 + (int)(Math.cos(ang + a2) * d), mPaint);
    c.drawLine(x2, y2, x2 + (int)(Math.sin(ang - a2) * d), y2 + (int)(Math.cos(ang - a2) * d), mPaint);
  }

  public void reset()
  {
    mPointerCount = 0;
    mHaveCoords = false;
    postInvalidate();
  }

  public void setGesture(int count, int x1, int y1, int x2, int y2)
  {
    //set coordinates
    if (count == 2)
    {
      mPointerX1 = x1;
      mPointerY1 = y1;
      mPointerX2 = x2;
      mPointerY2 = y2;
      if (mPointerCount == 1) //previous was 1 finger, current is two fingers -> ACTION_DOWN
      {
        mHaveCoords = true;
      }
    }
    mPointerCount = count;
    postInvalidate();
  }
}
