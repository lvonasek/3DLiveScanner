package com.lvonasek.arcore3dscanner.main;

import android.app.Activity;
import android.content.Context;
import android.util.AttributeSet;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.Transformation;
import android.widget.FrameLayout;
import android.widget.ImageView;

import com.lvonasek.arcore3dscanner.R;


/** This view contains the hand motion instructions with animation. */

public class HandMotionView extends ImageView {

  private static final long ANIMATION_SPEED_MS = 2500;

  public HandMotionView(Context context) {
    super(context);
  }

  public HandMotionView(Context context, AttributeSet attrs) {
    super(context, attrs);
  }

  @Override
  protected void onAttachedToWindow() {
    super.onAttachedToWindow();

    clearAnimation();

    FrameLayout container = ((Activity) getContext()).findViewById(R.id.ar_hand_layout);

    HandMotionAnimation animation = new HandMotionAnimation(container, this);
    animation.setRepeatCount(Animation.INFINITE);
    animation.setDuration(ANIMATION_SPEED_MS);
    animation.setStartOffset(1000);

    startAnimation(animation);
  }

  private static class HandMotionAnimation extends Animation {
    private final View handImageView;
    private final View containerView;
    private static final float TWO_PI = (float) Math.PI * 2.0f;
    private static final float HALF_PI = (float) Math.PI / 2.0f;

    public HandMotionAnimation(View containerView, View handImageView) {
      this.handImageView = handImageView;
      this.containerView = containerView;
    }

    @Override
    protected void applyTransformation(float interpolatedTime, Transformation transformation) {
      float progressAngle = TWO_PI * interpolatedTime;
      float currentAngle = HALF_PI + progressAngle;

      float handWidth = handImageView.getWidth();
      float radius = handImageView.getResources().getDisplayMetrics().density * 25.0f;

      float xPos = radius * 2.0f * (float) Math.cos(currentAngle);
      float yPos = radius * (float) Math.sin(currentAngle);

      xPos += containerView.getWidth() / 2.0f;
      yPos += containerView.getHeight() / 2.0f;

      xPos -= handWidth / 2.0f;
      yPos -= handImageView.getHeight() / 2.0f;

      // Position the hand.
      handImageView.setX(xPos);
      handImageView.setY(yPos);

      handImageView.invalidate();
    }
  }
}
