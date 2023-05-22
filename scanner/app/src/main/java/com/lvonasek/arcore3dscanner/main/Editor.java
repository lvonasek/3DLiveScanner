package com.lvonasek.arcore3dscanner.main;

import android.app.AlertDialog;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;

import com.lvonasek.arcore3dscanner.ui.AbstractActivity;
import com.lvonasek.arcore3dscanner.R;

import java.io.File;
import java.util.ArrayList;

public class Editor extends View implements Button.OnClickListener, View.OnTouchListener {

  private final int BUTTON_SAVE = 0;
  private final int BUTTON_SUBMENU_SELECT = 5;
  private final int BUTTON_SUBMENU_COLORS = 11;
  private final int BUTTON_SUBMENU_TRANSFORM = 16;
  private final int BUTTON_SUBMENU_VIEW = 19;
  private final int BUTTON_X = 23;
  private final int BUTTON_Y = 24;
  private final int BUTTON_Z = 25;

  private enum Effect { CONTRAST, GAMMA, SATURATION, TONE, RESET, CLONE, DELETE, MOVE, ROTATE, SCALE }
  private enum Screen { MAIN, COLOR, SELECT, TRANSFORM, EDIT}
  private enum Status { IDLE, SELECT_OBJECT, SELECT_CIRCLE, SELECT_RECT, UPDATE_COLORS, UPDATE_TRANSFORM }

  private int mAxis;
  private ArrayList<Button> mButtons;
  private AbstractActivity mContext;
  private Effect mEffect;
  private ProgressBar mProgress;
  private Screen mScreen;
  private SeekBar mSeek;
  private Status mStatus;
  private TextView mMsg;
  private Paint mPaint;
  private Rect mRect;
  private CheckBox mDeselect;
  private Point mCircleCenter;
  private float mCircleRadius;
  private boolean mShowNormals;

  private boolean mBackShown;
  private boolean mComplete;
  private boolean mInitialized;

  public Editor(Context context, AttributeSet attrs)
  {
    super(context, attrs);
    mInitialized = false;
    mPaint = new Paint();
    mPaint.setColor(0x8080FF80);
    mRect = new Rect();
    mCircleCenter = new Point();
    mCircleRadius = 0;
    mShowNormals = false;
  }

  public void init(ArrayList<Button> buttons, TextView msg, SeekBar seek, ProgressBar progress, CheckBox deselect, AbstractActivity context)
  {
    for (Button b : buttons) {
      b.setOnClickListener(this);
      b.setOnTouchListener(this);
    }
    mAxis = 1;
    mButtons = buttons;
    mContext = context;
    mDeselect = deselect;
    mMsg = msg;
    mProgress = progress;
    mSeek = seek;
    setMainScreen();

    mComplete = true;
    mInitialized = true;
    mProgress.setVisibility(View.VISIBLE);
    mSeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener()
    {
      @Override
      public void onProgressChanged(SeekBar seekBar, int value, boolean byUser)
      {
        if ((mStatus == Status.UPDATE_COLORS) || (mStatus == Status.UPDATE_TRANSFORM)) {
          value -= 127;
          JNI.previewEffect(mEffect.ordinal(), value, mAxis);
        }
      }

      @Override
      public void onStartTrackingTouch(SeekBar seekBar)
      {
      }

      @Override
      public void onStopTrackingTouch(SeekBar seekBar)
      {
      }
    });
    new Thread(() -> {
      JNI.completeSelection(mComplete);
      mContext.runOnUiThread(() -> mProgress.setVisibility(View.GONE));
    }).start();
  }

  private void applyTransform()
  {
    final int axis = mAxis;
    mProgress.setVisibility(View.VISIBLE);
    new Thread(() -> {
      JNI.applyEffect(mEffect.ordinal(), mSeek.getProgress() - 127, axis);
      mContext.runOnUiThread(() -> mProgress.setVisibility(View.INVISIBLE));
    }).start();
  }

  public boolean initialized() { return mInitialized; }

  public boolean movingLocked()
  {
    return mStatus != Status.IDLE;
  }

  private Rect normalizeRect(Rect input)
  {
    Rect output = new Rect();
    if (input.left > input.right) {
      output.left = input.right;
      output.right = input.left;
    } else {
      output.left = input.left;
      output.right = input.right;
    }

    if (input.top > input.bottom) {
      output.top = input.bottom;
      output.bottom = input.top;
    } else {
      output.top = input.top;
      output.bottom = input.bottom;
    }
    return output;
  }

  @Override
  public void onClick(final View view)
  {
    //axis buttons
    if (view.getId() == R.id.editorX) {
      applyTransform();
      mAxis = 0;
      showSeekBar(true);
    }
    if (view.getId() == R.id.editorY) {
      applyTransform();
      mAxis = 1;
      showSeekBar(true);
    }
    if (view.getId() == R.id.editorZ) {
      applyTransform();
      mAxis = 2;
      showSeekBar(true);
    }

    //back button
    if (view.getId() == R.id.editor0) {
      if ((mStatus == Status.SELECT_OBJECT) || (mStatus == Status.SELECT_CIRCLE) || (mStatus == Status.SELECT_RECT)) {
        mDeselect.setVisibility(View.GONE);
        setMainScreen();
      } else if (mStatus == Status.UPDATE_COLORS) {
        mProgress.setVisibility(View.VISIBLE);
        new Thread(() -> {
            JNI.applyEffect(mEffect.ordinal(), mSeek.getProgress() - 127, 0);
            mContext.runOnUiThread(() -> mProgress.setVisibility(View.INVISIBLE));
          }).start();
        setMainScreen();
      } else if (mStatus == Status.UPDATE_TRANSFORM) {
        applyTransform();
        setMainScreen();
      } else
        save();
    }
    //main menu
    else if (view.getId() == R.id.editor1)
      setSelectScreen();
    else if (view.getId() == R.id.editor2)
      setColorScreen();
    else if (view.getId() == R.id.editor3)
      setTransformScreen();
    else if (view.getId() == R.id.editor4)
      setViewScreen();

    //selecting objects
    if (mScreen == Screen.SELECT) {
      //select all/none
      if (view.getId() == R.id.editor1a)
      {
        mProgress.setVisibility(View.VISIBLE);
        new Thread(() -> {
          mComplete = !mComplete;
          JNI.completeSelection(mComplete);
          mContext.runOnUiThread(() -> mProgress.setVisibility(View.GONE));
        }).start();
      }
      //select object
      if (view.getId() == R.id.editor1b) {
        showText(R.string.editor_select_object_desc);
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        mStatus = Status.SELECT_OBJECT;
      }
      //rect selection
      if (view.getId() == R.id.editor1c) {
        showText(R.string.editor_select_circle_desc);
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        mDeselect.setVisibility(View.VISIBLE);
        mStatus = Status.SELECT_CIRCLE;
      }
      //rect selection
      if (view.getId() == R.id.editor1d) {
        showText(R.string.editor_select_rect_desc);
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        mDeselect.setVisibility(View.VISIBLE);
        mStatus = Status.SELECT_RECT;
      }
      //select less
      if (view.getId() == R.id.editor1e)
      {
        mProgress.setVisibility(View.VISIBLE);
        new Thread(() -> {
          JNI.multSelection(false);
          mContext.runOnUiThread(() -> mProgress.setVisibility(View.GONE));
        }).start();
      }
      //select more
      if (view.getId() == R.id.editor1f)
      {
        mProgress.setVisibility(View.VISIBLE);
        new Thread(() -> {
          JNI.multSelection(true);
          mContext.runOnUiThread(() -> mProgress.setVisibility(View.GONE));
        }).start();
      }
    }

    //color editing
    if (mScreen == Screen.COLOR) {
      if (view.getId() != R.id.editor0) {
        if (mShowNormals)
        {
          mShowNormals = false;
          JNI.showNormals(false);
        }
      }

      if (view.getId() == R.id.editor2a)
      {
        mEffect = Effect.CONTRAST;
        mStatus = Status.UPDATE_COLORS;
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        showSeekBar(false);
      }
      if (view.getId() == R.id.editor2b)
      {
        mEffect = Effect.GAMMA;
        mStatus = Status.UPDATE_COLORS;
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        showSeekBar(false);
      }
      if (view.getId() == R.id.editor2c)
      {
        mEffect = Effect.SATURATION;
        mStatus = Status.UPDATE_COLORS;
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        showSeekBar(false);
      }
      if (view.getId() == R.id.editor2d)
      {
        mEffect = Effect.TONE;
        mStatus = Status.UPDATE_COLORS;
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        showSeekBar(false);
      }
      if (view.getId() == R.id.editor2e)
      {
        mProgress.setVisibility(View.VISIBLE);
        new Thread(() -> {
          JNI.applyEffect(Effect.RESET.ordinal(), 0, 0);
          mContext.runOnUiThread(() -> mProgress.setVisibility(View.INVISIBLE));
        }).start();
      }
    }

    // transforming objects
    if (mScreen == Screen.TRANSFORM) {
      if (view.getId() == R.id.editor3a)
      {
        mEffect = Effect.MOVE;
        mStatus = Status.UPDATE_TRANSFORM;
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        showSeekBar(true);
      }
      if (view.getId() == R.id.editor3b)
      {
        mEffect = Effect.ROTATE;
        mStatus = Status.UPDATE_TRANSFORM;
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        showSeekBar(true);
      }
      if (view.getId() == R.id.editor3c)
      {
        mEffect = Effect.SCALE;
        mStatus = Status.UPDATE_TRANSFORM;
        mBackShown = true;
        mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_back_small);
        showSeekBar(false);
      }
    }

    //view
    if (mScreen == Screen.EDIT) {
      if (view.getId() == R.id.editor4a)
      {
        mProgress.setVisibility(View.VISIBLE);
        new Thread(() -> {
          JNI.restore();
          mContext.runOnUiThread(() -> mProgress.setVisibility(View.INVISIBLE));
        }).start();
      }
      if (view.getId() == R.id.editor4b)
      {
        mProgress.setVisibility(View.VISIBLE);
        new Thread(() -> {
          JNI.applyEffect(Effect.CLONE.ordinal(), 0, 0);
          mContext.runOnUiThread(() -> mProgress.setVisibility(View.INVISIBLE));
        }).start();
      }
      if (view.getId() == R.id.editor4c)
      {
        mProgress.setVisibility(View.VISIBLE);
        new Thread(() -> {
          JNI.applyEffect(Effect.DELETE.ordinal(), 0, 0);
          mContext.runOnUiThread(() -> mProgress.setVisibility(View.INVISIBLE));
        }).start();
      }
      if (view.getId() == R.id.editor4d)
      {
        swapNormals();
      }
    }
  }

  @Override
  protected void onDraw(Canvas c)
  {
    super.onDraw(c);
    c.drawCircle(mCircleCenter.x, mCircleCenter.y, mCircleRadius, mPaint);
    c.drawRect(normalizeRect(mRect), mPaint);
  }

  @Override
  public boolean onTouch(View view, MotionEvent motionEvent)
  {
    if (view instanceof Button) {
      Button b = (Button) view;
      if (motionEvent.getAction() == MotionEvent.ACTION_DOWN)
        b.setTextColor(Color.YELLOW);
      if (motionEvent.getAction() == MotionEvent.ACTION_UP)
        b.setTextColor(Color.WHITE);
    }
    return false;
  }

  private void initButtons()
  {
    mMsg.setVisibility(View.GONE);
    mSeek.setVisibility(View.GONE);
    mStatus = Status.IDLE;
    for (Button b : mButtons) {
      b.setVisibility(View.VISIBLE);
    }
    mButtons.get(BUTTON_X).setVisibility(View.GONE);
    mButtons.get(BUTTON_Y).setVisibility(View.GONE);
    mButtons.get(BUTTON_Z).setVisibility(View.GONE);
    mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_save_small);
    mBackShown = false;
  }

  private void save() {
    //filename dialog
    AlertDialog.Builder builder = new AlertDialog.Builder(mContext);
    builder.setTitle(mContext.getString(R.string.enter_filename));
    final EditText input = new EditText(mContext);
    builder.setView(input);
    builder.setPositiveButton(mContext.getString(android.R.string.ok), (dialog, which) -> {
      //delete old during overwrite
      File file = new File(AbstractActivity.getPath(false), input.getText().toString() + Exporter.EXT_OBJ);
      try {
        if (file.exists())
          for(String s : Exporter.getObjResources(file))
            if (new File(AbstractActivity.getPath(false), s).delete())
              Log.d(AbstractActivity.TAG, "File " + s + " deleted");
      } catch(Exception e) {
        e.printStackTrace();
      }
      mProgress.setVisibility(View.VISIBLE);
      new Thread(() -> {
        long timestamp = System.currentTimeMillis();
        final File obj = new File(AbstractActivity.getTempPath(), timestamp + Exporter.EXT_OBJ);
        JNI.saveWithTextures(obj.getAbsolutePath().getBytes());
        for(String s : Exporter.getObjResources(obj.getAbsoluteFile()))
          if (new File(AbstractActivity.getTempPath(), s).renameTo(new File(AbstractActivity.getPath(false), s)))
            Log.d(AbstractActivity.TAG, "File " + s + " saved");
        final File file2save = new File(AbstractActivity.getPath(false), input.getText().toString() + Exporter.EXT_OBJ);
        if (obj.renameTo(file2save))
          Log.d(AbstractActivity.TAG, "Obj file " + file2save.toString() + " saved.");
        mContext.runOnUiThread(() -> mProgress.setVisibility(View.GONE));
      }).start();
      dialog.cancel();
    });
    builder.setNegativeButton(mContext.getString(android.R.string.cancel), null);
    builder.create().show();
  }

  private void setMainScreen()
  {
    initButtons();
    mBackShown = false;
    mButtons.get(BUTTON_SAVE).setBackgroundResource(R.drawable.ic_save_small);
    for (int i = BUTTON_SUBMENU_SELECT; i < mButtons.size(); i++) {
      mButtons.get(i).setVisibility(View.GONE);
    }
    mScreen = Screen.MAIN;
  }

  private void setColorScreen()
  {
    initButtons();
    for (int i = BUTTON_SUBMENU_SELECT; i < mButtons.size(); i++) {
      mButtons.get(i).setVisibility(i >= BUTTON_SUBMENU_COLORS && i < BUTTON_SUBMENU_TRANSFORM ? View.VISIBLE : View.GONE);
    }
    mScreen = Screen.COLOR;
  }

  private void setSelectScreen()
  {
    initButtons();
    for (int i = BUTTON_SUBMENU_SELECT; i < mButtons.size(); i++) {
      mButtons.get(i).setVisibility(i < BUTTON_SUBMENU_COLORS ? View.VISIBLE : View.GONE);
    }
    mScreen = Screen.SELECT;
  }

  private void setTransformScreen()
  {
    initButtons();
    for (int i = BUTTON_SUBMENU_SELECT; i < mButtons.size(); i++) {
      mButtons.get(i).setVisibility(i >= BUTTON_SUBMENU_TRANSFORM && i < BUTTON_SUBMENU_VIEW ? View.VISIBLE : View.GONE);
    }
    mScreen = Screen.TRANSFORM;
  }

  private void setViewScreen()
  {
    initButtons();
    for (int i = BUTTON_SUBMENU_SELECT; i < mButtons.size(); i++) {
      mButtons.get(i).setVisibility(i >= BUTTON_SUBMENU_VIEW && i < BUTTON_X ? View.VISIBLE : View.GONE);
    }
    mScreen = Screen.EDIT;
  }

  private void showSeekBar(boolean axes)
  {
    for (Button b : mButtons)
      b.setVisibility(View.GONE);
    mButtons.get(BUTTON_SAVE).setVisibility(View.VISIBLE);
    if (axes)
      updateAxisButtons();
    mSeek.setMax(255);
    mSeek.setProgress(127);
    mSeek.setVisibility(View.VISIBLE);
  }

  private void showText(int resId)
  {
    for (Button b : mButtons)
      b.setVisibility(View.GONE);
    mButtons.get(BUTTON_SAVE).setVisibility(View.VISIBLE);
    mMsg.setText(mContext.getString(resId));
    mMsg.setVisibility(View.VISIBLE);
  }

  public void swapNormals()
  {
    mShowNormals = !mShowNormals;
    JNI.showNormals(mShowNormals);
  }

  public void touchEvent(final MotionEvent event)
  {
    if (!mBackShown) {
      setMainScreen();
      return;
    }

    if (mStatus == Status.SELECT_OBJECT) {
      mProgress.setVisibility(View.VISIBLE);
      new Thread(() -> {
        JNI.applySelect(event.getX(), getHeight() - event.getY(), false);
        mContext.runOnUiThread(() -> mProgress.setVisibility(View.GONE));
      }).start();
      mStatus = Status.IDLE;
      setSelectScreen();
    }

    if (mStatus == Status.SELECT_CIRCLE) {
      if (event.getAction() == MotionEvent.ACTION_DOWN) {
        mCircleCenter.x = (int) event.getX();
        mCircleCenter.y = (int) event.getY();
        mCircleRadius = 0;
      }
      if (event.getAction() == MotionEvent.ACTION_MOVE) {
        float dx = mCircleCenter.x - (int) event.getX();
        float dy = mCircleCenter.y - (int) event.getY();
        mCircleRadius = (float) Math.sqrt(dx * dx + dy * dy);
      }
      if (event.getAction() == MotionEvent.ACTION_UP) {
        float dx = mCircleCenter.x - (int) event.getX();
        float dy = mCircleCenter.y - (int) event.getY();
        mCircleCenter.y = getHeight() - mCircleCenter.y;
        mCircleRadius = (float) Math.sqrt(dx * dx + dy * dy);
        JNI.circleSelection(mCircleCenter.x, mCircleCenter.y, mCircleRadius, mDeselect.isChecked());
        mCircleCenter = new Point();
        mCircleRadius = 0;
      }
      postInvalidate();
    }

    if (mStatus == Status.SELECT_RECT) {
      if (event.getAction() == MotionEvent.ACTION_DOWN) {
        mRect.left = (int) event.getX();
        mRect.top = (int) event.getY();
        mRect.right = mRect.left;
        mRect.bottom = mRect.top;
      }
      if (event.getAction() == MotionEvent.ACTION_MOVE) {
        mRect.right = (int) event.getX();
        mRect.bottom = (int) event.getY();
      }
      if (event.getAction() == MotionEvent.ACTION_UP) {
        Rect rect = normalizeRect(mRect);
        rect.top = getHeight() - rect.top;
        rect.bottom = getHeight() - rect.bottom;
        JNI.rectSelection(rect.left, rect.bottom, rect.right, rect.top, mDeselect.isChecked());
        mRect = new Rect();
      }
      postInvalidate();
    }
  }

  private void updateAxisButtons()
  {
    mButtons.get(BUTTON_X).setVisibility(View.VISIBLE);
    mButtons.get(BUTTON_Y).setVisibility(View.VISIBLE);
    mButtons.get(BUTTON_Z).setVisibility(View.VISIBLE);
    mButtons.get(BUTTON_X).setText("X");
    mButtons.get(BUTTON_Y).setText("Y");
    mButtons.get(BUTTON_Z).setText("Z");
    mButtons.get(BUTTON_X).setTextColor(mAxis == 0 ? Color.BLACK : Color.WHITE);
    mButtons.get(BUTTON_Y).setTextColor(mAxis == 1 ? Color.BLACK : Color.WHITE);
    mButtons.get(BUTTON_Z).setTextColor(mAxis == 2 ? Color.BLACK : Color.WHITE);
    mButtons.get(BUTTON_X).setBackgroundColor(mAxis == 0 ? Color.WHITE : Color.TRANSPARENT);
    mButtons.get(BUTTON_Y).setBackgroundColor(mAxis == 1 ? Color.WHITE : Color.TRANSPARENT);
    mButtons.get(BUTTON_Z).setBackgroundColor(mAxis == 2 ? Color.WHITE : Color.TRANSPARENT);
  }
}
