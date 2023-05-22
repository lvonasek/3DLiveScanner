package com.lvonasek.arcore3dscanner.ui;

import android.content.Context;
import android.graphics.drawable.Drawable;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.ImageView;
import android.widget.TextView;

import com.lvonasek.arcore3dscanner.R;

import java.util.List;

public class ArrayAdapterWithIcons extends BaseAdapter {

    private Context context;
    private List<Drawable> icons;
    private List<String> strings;

    public ArrayAdapterWithIcons(Context context, List<String> strings, List<Drawable> icons) {
        this.context = context;
        this.icons = icons;
        this.strings = strings;
    }

    @Override
    public int getCount() {
        return strings.size();
    }

    @Override
    public Object getItem(int i) {
        return strings.get(i);
    }

    @Override
    public long getItemId(int i) {
        return i;
    }

    @Override
    public View getView(int i, View view, ViewGroup viewGroup) {
        LayoutInflater inflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        view = inflater.inflate(R.layout.view_texticon, null, true);

        TextView text = view.findViewById(R.id.name);
        text.setText(strings.get(i));

        ImageView icon = view.findViewById(R.id.icon);
        icon.setImageDrawable(icons.get(i));
        return view;
    }
}
