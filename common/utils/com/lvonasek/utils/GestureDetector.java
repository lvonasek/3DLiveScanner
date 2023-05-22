package com.lvonasek.utils;

import android.content.Context;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

public class GestureDetector
{
  private static final int INVALID_ANGLE = 9999;
  private static final int INVALID_POINTER_ID = -1;
  private float fX, fY, sX, sY;
  private float frX, frY, srX, srY;
  private int ptrID1, ptrID2;
  private float mAngle = 0;
  private float mLastAngle = INVALID_ANGLE + 1;
  private float mMoveX = 0;
  private float mMoveY = 0;
  private boolean mMoveValid = false;

  private GestureListener mListener;
  private ScaleGestureDetector mScaleDetector;

  public GestureDetector(GestureListener listener, Context context)
  {
    mListener = listener;
    ptrID1 = INVALID_POINTER_ID;
    ptrID2 = INVALID_POINTER_ID;

    mScaleDetector = new ScaleGestureDetector(context, new ScaleGestureDetector.OnScaleGestureListener() {

      float last;

      @Override
      public void onScaleEnd(ScaleGestureDetector detector) {
      }
      @Override
      public boolean onScaleBegin(ScaleGestureDetector detector) {
        last = 0;
        return true;
      }
      @Override
      public boolean onScale(ScaleGestureDetector detector) {
        if (mListener != null) {
          float f = detector.getScaleFactor() - 1.0f;
          if (f > 0)
            mListener.OnPinchToZoom((f - last) * 4.0f);
          else
            mListener.OnPinchToZoom((f - last) * 8.0f);
          last = f;
        }
        return false;
      }
    });
  }

  public void onTouchEvent(MotionEvent event)
  {
    mScaleDetector.onTouchEvent(event);
    switch (event.getActionMasked())
    {
      case MotionEvent.ACTION_DOWN:
        ptrID1 = event.getPointerId(event.getActionIndex());
        mMoveX = event.getRawX();
        mMoveY = event.getRawY();
        mMoveValid = true;
        break;
      case MotionEvent.ACTION_POINTER_DOWN:
        ptrID2 = event.getPointerId(event.getActionIndex());
        srX = sX = event.getX(event.findPointerIndex(ptrID1));
        srY = sY = event.getY(event.findPointerIndex(ptrID1));
        frX = fX = event.getX(event.findPointerIndex(ptrID2));
        frY = fY = event.getY(event.findPointerIndex(ptrID2));
        mMoveValid = false;
        break;
      case MotionEvent.ACTION_MOVE:
        if (ptrID1 != INVALID_POINTER_ID && ptrID2 != INVALID_POINTER_ID)
        {
          float nfX, nfY, nsX, nsY;
          nsX = event.getX(event.findPointerIndex(ptrID1));
          nsY = event.getY(event.findPointerIndex(ptrID1));
          nfX = event.getX(event.findPointerIndex(ptrID2));
          nfY = event.getY(event.findPointerIndex(ptrID2));

          if (mListener.IsAcceptingRotation()) {

            float angle = angleBetweenLines(fX, fY, sX, sY, nfX, nfY, nsX, nsY);
            float gap = angle - mLastAngle;
            while(gap > 180)
              gap -= 360.0f;
            while(gap < -180)
              gap += 360.0f;

            if(mLastAngle < INVALID_ANGLE)
              mAngle += gap;
            mLastAngle = angle;

            if (mListener != null)
              mListener.OnTwoFingerRotation(mAngle);
          }

          float dfX = frX - nfX;
          float dfY = frY - nfY;
          float dsX = srX - nsX;
          float dsY = srY - nsY;

          mListener.OnTwoFingerMove((dsX + dfX) * 0.5f, -(dsY + dfY) * 0.5f);
          frX = nfX;
          frY = nfY;
          srX = nsX;
          srY = nsY;
        }
        else if (mMoveValid)
        {
          if (mListener != null)
            mListener.OnDrag(mMoveX - event.getRawX(), event.getRawY() - mMoveY);
          mMoveX = event.getRawX();
          mMoveY = event.getRawY();
        }
        break;
      case MotionEvent.ACTION_UP:
        ptrID1 = INVALID_POINTER_ID;
        mLastAngle = INVALID_ANGLE + 1;
        mMoveValid = false;
        break;
      case MotionEvent.ACTION_POINTER_UP:
        ptrID2 = INVALID_POINTER_ID;
        mLastAngle = INVALID_ANGLE + 1;
        mMoveValid = false;
        break;
      case MotionEvent.ACTION_CANCEL:
        ptrID1 = INVALID_POINTER_ID;
        ptrID2 = INVALID_POINTER_ID;
        mLastAngle = INVALID_ANGLE + 1;
        mMoveValid = false;
        break;
    }
  }


  public GestureListener listener() {
    return mListener;
  }

  private float angleBetweenLines(float fX, float fY, float sX, float sY, float nfX, float nfY, float nsX, float nsY)
  {
    float angle1 = (float) Math.atan2((fY - sY), (fX - sX));
    float angle2 = (float) Math.atan2((nfY - nsY), (nfX - nsX));

    float angle = ((float) Math.toDegrees(angle1 - angle2)) % 360;
    if (angle < -180.f) angle += 360.0f;
    if (angle > 180.f) angle -= 360.0f;
    return angle;
  }

  public interface GestureListener
  {
    boolean IsAcceptingRotation();

    void OnDrag(float dx, float dy);

    void OnTwoFingerMove(float dx, float dy);

    void OnTwoFingerRotation(float angle);

    void OnPinchToZoom(float diff);
  }
}