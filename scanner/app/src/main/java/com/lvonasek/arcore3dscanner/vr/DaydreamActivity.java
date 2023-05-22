package com.lvonasek.arcore3dscanner.vr;

import com.google.vr.sdk.base.Viewport;
import com.google.vr.sdk.controller.Controller;
import com.google.vr.sdk.controller.ControllerManager;

public class DaydreamActivity extends VRActivity implements Runnable
{
  private ControllerManager controllerManager;
  private float addYaw = 0;
  private float speed = 0.05f;
  private long timestamp;

  @Override
  protected void onStart() {
    super.onStart();
    controllerManager = new ControllerManager(this, new ControllerManager.EventListener()
    {
      @Override
      public void onApiStatusChanged(int i)
      {
      }

      @Override
      public void onRecentered()
      {
        yaw = 0;
      }
    });
    controllerManager.start();
  }

  @Override
  protected void onStop() {
    controllerManager.stop();
    controllerManager = null;
    super.onStop();
  }

  @Override
  public synchronized void onFinishFrame(Viewport viewport) {
    runOnUiThread(this);
    if (ready)
      nativeUpdate();
  }

  @Override
  public synchronized void run()
  {
    if (controllerManager == null)
      return;
    if (System.currentTimeMillis() < timestamp)
      return;
    Controller controller = controllerManager.getController();
    controller.update();
    if (controller.clickButtonState && ready)
      nativeOnTriggerEvent(0, 0, speed, headView);
    if (controller.appButtonState) {
      timestamp = System.currentTimeMillis() + 200;
      yaw += 90;
    }
    if (controller.volumeUpButtonState)
      speed *= 1.25f;
    if (controller.volumeDownButtonState)
      speed /= 1.25f;
    addYaw *= 0.95f;
    yaw -= 2.0f * addYaw;
  }
}
