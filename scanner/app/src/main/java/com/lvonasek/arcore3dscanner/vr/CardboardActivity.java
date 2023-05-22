package com.lvonasek.arcore3dscanner.vr;

import android.view.KeyEvent;

import com.google.vr.sdk.base.Viewport;

public class CardboardActivity extends VRActivity
{
  private float dX = 0;
  private float dZ = 0;

  @Override
  public void onCardboardTrigger() {
    if (ready)
      nativeOnTriggerEvent(0, 0, 1, headView);
  }

  @Override
  public synchronized void onFinishFrame(Viewport viewport) {
    if (ready)
      nativeUpdate();
    nativeOnTriggerEvent(dX, 0, dZ, headView);
  }

  @Override
  public boolean onKeyDown(int keyCode, KeyEvent keyEvent) {
    float fwdStep = 0.05f;
    float sideStep = 0.01f;
    switch (keyCode) {
      case KeyEvent.KEYCODE_DPAD_LEFT:
        dX = sideStep;
        break;
      case KeyEvent.KEYCODE_DPAD_RIGHT:
        dX = -sideStep;
        break;
      case KeyEvent.KEYCODE_DPAD_DOWN:
        dZ = -fwdStep;
        break;
      case KeyEvent.KEYCODE_DPAD_UP:
        dZ = fwdStep;
        break;
    }
    return true;
  }

  @Override
  public boolean onKeyUp(int keyCode, KeyEvent keyEvent) {
    switch (keyCode) {
      case KeyEvent.KEYCODE_DPAD_LEFT:
      case KeyEvent.KEYCODE_DPAD_RIGHT:
        dX = 0;
        break;
      case KeyEvent.KEYCODE_DPAD_DOWN:
      case KeyEvent.KEYCODE_DPAD_UP:
        dZ = 0;
        break;
    }
    return true;
  }
}