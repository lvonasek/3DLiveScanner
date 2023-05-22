package com.lvonasek.arcore3dscanner.ui;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;

public class CommonDialogs {

    public static void confirmDialog(Activity context, int title, Runnable proceed) {
        AlertDialog.Builder dialog = new AlertDialog.Builder(context);
        dialog.setTitle(title);
        dialog.setMessage(com.lvonasek.arcore3dscanner.R.string.continue_question);
        dialog.setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                proceed.run();
            }
        });
        dialog.setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });
        AlertDialog d = dialog.create();
        d.getWindow().setFlags(WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE, WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE);
        d.getWindow().setBackgroundDrawableResource(com.lvonasek.arcore3dscanner.R.drawable.background_dialog);
        d.setOnDismissListener(dialogInterface -> setImmersive(context.getWindow()));
        d.show();

        //workaround to system UI glitch
        setImmersive(d, context);
    }

    public static void setImmersive(AlertDialog d, Activity context) {
        d.getWindow().setFlags(WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE, WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE);
        d.getWindow().getDecorView().setSystemUiVisibility(context.getWindow().getDecorView().getSystemUiVisibility());
        d.setOnShowListener(dialog1 -> {
            d.getWindow().clearFlags(WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE);
            WindowManager wm = (WindowManager) context.getSystemService(Context.WINDOW_SERVICE);
            wm.updateViewLayout(d.getWindow().getDecorView(), d.getWindow().getAttributes());
        });
    }

    public static void setImmersive(Window window) {
        window.peekDecorView().setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }
}
