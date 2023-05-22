package com.lvonasek.utils;

import android.util.Log;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

public class IO {

    private static final int BUFFER_SIZE = 65536;
    private static final String TAG = "arcore_app";

    public static void copy(File src, File dst) {
        try (FileInputStream in = new FileInputStream(src)) {
            try (FileOutputStream out = new FileOutputStream(dst)) {
                byte[] buf = new byte[1024];
                int len;
                while ((len = in.read(buf)) > 0) {
                    out.write(buf, 0, len);
                }
                out.close();
                in.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void deleteRecursive(File fileOrDirectory) {
        try {
            if (fileOrDirectory.isDirectory())
                for (File child : fileOrDirectory.listFiles())
                    deleteRecursive(child);

            if (fileOrDirectory.delete())
                Log.d(TAG, fileOrDirectory + " deleted");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static boolean unzip(String path, InputStream is) {
        try {
            String filename;
            ZipInputStream zis = new ZipInputStream(new BufferedInputStream(is));
            ZipEntry ze;
            byte[] buffer = new byte[BUFFER_SIZE];
            int count;

            while ((ze = zis.getNextEntry()) != null) {
                filename = ze.getName();

                //https://support.google.com/faqs/answer/9294009
                File file = new File(path, filename);
                String canonicalPath = file.getCanonicalPath();
                if (!canonicalPath.startsWith(path)) {
                    String canonical = canonicalPath.replace("/data/data/", "/data/user/0/");
                    if (!canonical.startsWith(path)) {
                        throw new SecurityException();
                    }
                }

                // Need to create directories if not exists, or
                // it will generate an Exception...
                if (ze.isDirectory()) {
                    if (file.mkdirs()) {
                        Log.d(TAG, "Directory" + file.getAbsolutePath() + " created");
                    }
                    continue;
                }

                if (!file.getParentFile().exists()) {
                    if (file.getParentFile().mkdirs()) {
                        Log.d(TAG, "Directory" + file.getAbsolutePath() + " created");
                    }
                }

                FileOutputStream fout = new FileOutputStream(file.getAbsolutePath());
                while ((count = zis.read(buffer)) != -1) {
                    fout.write(buffer, 0, count);
                }

                fout.close();
                zis.closeEntry();
            }
            zis.close();
        }
        catch(Exception e)
        {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public static void zip(List<String> files, String zip) throws Exception {
        try (ZipOutputStream out = new ZipOutputStream(new BufferedOutputStream(new FileOutputStream(zip))))
        {
            out.setMethod(ZipOutputStream.DEFLATED);
            out.setLevel(9);
            byte data[] = new byte[BUFFER_SIZE];
            for (String file : files)
            {
                FileInputStream fi = new FileInputStream(file);
                try (BufferedInputStream origin = new BufferedInputStream(fi, BUFFER_SIZE))
                {
                    ZipEntry entry = new ZipEntry(file.substring(file.lastIndexOf("/") + 1));
                    out.putNextEntry(entry);
                    int count;
                    while ((count = origin.read(data, 0, BUFFER_SIZE)) != -1)
                    {
                        out.write(data, 0, count);
                    }
                }
            }
        }
    }
}
