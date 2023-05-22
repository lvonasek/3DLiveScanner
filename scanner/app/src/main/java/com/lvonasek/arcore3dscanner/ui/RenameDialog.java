package com.lvonasek.arcore3dscanner.ui;

import android.app.AlertDialog;
import android.util.Log;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import com.lvonasek.arcore3dscanner.main.Exporter;
import com.lvonasek.arcore3dscanner.R;

import java.io.File;

public class RenameDialog {

    private EditText mInput;
    private ListView mList;
    private TextView mPath;

    public RenameDialog(FileManager context, String path, String key) {
        AlertDialog.Builder renameDlg = new AlertDialog.Builder(context);
        renameDlg.setView(com.lvonasek.arcore3dscanner.R.layout.dialog_rename);
        renameDlg.setPositiveButton(context.getString(android.R.string.ok), (dialog, which) -> {
            int type = Exporter.getModelType(key);
            String name1 = mInput.getText().toString();
            if (type >= 0) {
                name1 = name1 + Exporter.FILE_EXT[type];
            }
            File newFile = new File(mPath.getText().toString(), name1);
            if(newFile.exists())
                Toast.makeText(context, com.lvonasek.arcore3dscanner.R.string.name_exists, Toast.LENGTH_LONG).show();
            else {
                File oldFile = new File(path, key);
                if (oldFile.renameTo(newFile))
                    Log.d(AbstractActivity.TAG, "File " + oldFile + " renamed to " + newFile);
                context.refreshUI();
            }
        });
        renameDlg.setNegativeButton(context.getString(android.R.string.cancel), null);

        AlertDialog d = renameDlg.create();
        d.getWindow().setBackgroundDrawable(context.getDrawable(com.lvonasek.arcore3dscanner.R.drawable.background_dialog));
        d.show();

        String name = key;
        if (name.contains(".")) {
            name = name.substring(0, name.lastIndexOf('.'));
        }
        mInput = d.findViewById(R.id.filename);
        mInput.setText(name);
        mPath = d.findViewById(R.id.path);
        mPath.setText(path);

        FolderAdapter adapter = new FolderAdapter(context, path, p -> mPath.setText(p));
        mList = d.findViewById(R.id.list);
        mList.setAdapter(adapter);
        adapter.update();
    }
}
