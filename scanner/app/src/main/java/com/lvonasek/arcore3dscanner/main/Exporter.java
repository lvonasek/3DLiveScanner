package com.lvonasek.arcore3dscanner.main;

import android.util.Log;

import com.lvonasek.arcore3dscanner.ui.AbstractActivity;
import com.lvonasek.utils.IO;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Scanner;

public class Exporter
{
  public static final String[] FILE_EXT = {".dataset", ".obj", ".ply"};
  public static final String EXT_DATASET = FILE_EXT[0];
  public static final String EXT_OBJ = FILE_EXT[1];
  public static final String EXT_PLY = FILE_EXT[2];

  public static final int EXPORT_TYPE_FLOORPLAN = -100;
  public static final int EXPORT_TYPE_POINTCLOUD = -200;

  public static String compressModel(File model2share) {
    ArrayList<String> filesToZip = new ArrayList<>();
    for (String s : model2share.list()) {
      File f = new File(model2share, s);
      if (!f.isDirectory()) {
        filesToZip.add(f.getAbsolutePath());
      }
    }

    File zipFile = new File(AbstractActivity.getTempPath(), "upload.scan.zip");
    if (zipFile.exists())
      zipFile.delete();
    String zipOutput = zipFile.getAbsolutePath();
    try {
      IO.zip(filesToZip, zipOutput);
    } catch (Exception e) {
      e.printStackTrace();
    }
    return zipOutput;
  }

  public static File export(File file, String filename) {
    File file2save = null;

    if (file.getAbsolutePath().endsWith(Exporter.EXT_OBJ)) {
      file2save = new File(AbstractActivity.getPath(false), filename + Exporter.EXT_OBJ);

      //delete old files during overwrite
      try {
        if (file2save.exists())
          for (String s : Exporter.getObjResources(file2save))
            if (new File(AbstractActivity.getPath(false), s).delete())
              Log.d(AbstractActivity.TAG, "File " + s + " deleted");
      } catch (Exception e) {
        e.printStackTrace();
      }

      //move file from temp into folder
      for (String s : Exporter.getObjResources(file.getAbsoluteFile()))
        if (new File(new File(file.getParent()), s).renameTo(new File(AbstractActivity.getPath(false), s)))
          Log.d(AbstractActivity.TAG, "File " + s + " saved");
      if (file.renameTo(file2save))
        Log.d(AbstractActivity.TAG, "Obj file " + file2save.toString() + " saved.");
    }

    if (file.getAbsolutePath().endsWith(Exporter.EXT_PLY)) {
      file2save = new File(AbstractActivity.getPath(false), filename + Exporter.EXT_PLY);

      //delete old file during overwrite
      if (file2save.exists())
        file2save.delete();

      //move file from temp
      if (file.renameTo(file2save))
        Log.d(AbstractActivity.TAG, "Ply file " + file2save.toString() + " saved.");
    }

    //copy GPS file
    File gpsFile = new File(file.getParent(), "position.txt");
    if (gpsFile.exists()) {
      File newGPS = new File(AbstractActivity.getPath(false), "position.txt");
      if (newGPS.exists()) {
        newGPS.delete();
      }
      IO.copy(gpsFile, newGPS);
    }

    return file2save;
  }

  public static boolean isFolder(String s) {
    for (String ext : Exporter.FILE_EXT) {
      if (s.endsWith(ext)) {
        return false;
      }
    }
    return true;
  }

  public static int getModelType(String filename) {
    for(int i = 0; i < FILE_EXT.length; i++) {
      int begin = filename.length() - FILE_EXT[i].length();
      if (begin >= 0)
        if (filename.substring(begin).contains(FILE_EXT[i]))
          return i;
    }
    return -1;
  }

  public static String getMtlResource(String obj)
  {
    try
    {
      Scanner sc = new Scanner(new FileInputStream(obj));
      while(sc.hasNext()) {
        String line = sc.nextLine();
        if (line.startsWith("mtllib")) {
          return line.substring(7);
        }
      }
      sc.close();
    } catch (Exception e)
    {
      e.printStackTrace();
    }
    return null;
  }

  public static ArrayList<String> getObjResources(File file)
  {
    HashSet<String> files = new HashSet<>();
    ArrayList<String> output = new ArrayList<>();
    String mtlLib = getMtlResource(file.getAbsolutePath());
    if (mtlLib != null) {
      output.add(mtlLib);
      output.add(mtlLib + ".png");
      mtlLib = file.getParent() + "/" + mtlLib;
      try
      {
        Scanner sc = new Scanner(new FileInputStream(mtlLib));
        while(sc.hasNext()) {
          String line = sc.nextLine();
          if (line.startsWith("map_") || line.startsWith("norm")) {
            String filename = line.substring(line.indexOf(" ") + 1);
            if (!files.contains(filename)) {
              files.add(filename);
              output.add(filename);
            }
          }
        }
        sc.close();
      } catch (Exception e)
      {
        e.printStackTrace();
      }
    }
    return output;
  }

  public static void makeStructure(String path) {
    //get list of files
    ArrayList<String> files = AbstractActivity.listFiles(new File(path));
    if (files.isEmpty())
      return;
    Collections.sort(files, String::compareTo);
    ArrayList<String> models = new ArrayList<>();
    for(String s : files)
      if(new File(s).isFile())
        if(getModelType(s) >= 0)
          models.add(s);

    //restructure models
    for (String s : models) {

      //create temp folder
      File temp = new File(path, "temp");
      if (temp.exists())
        AbstractActivity.deleteRecursive(temp);
      temp.mkdir();

      //get model files
      ArrayList<String> res = new ArrayList<>();
      File gpsFile = new File(path, "position.txt");
      if (gpsFile.exists())
        res.add(gpsFile.getName());
      if (Exporter.FILE_EXT[getModelType(s)].compareTo(Exporter.EXT_OBJ) == 0) {
        res.addAll(getObjResources(new File(s)));
      }
      res.add(new File(s).getName());

      //restructure
      for (String r : res) {
        File f = new File(path, r);
        if (f.exists())
          f.renameTo(new File(temp, r));
      }
      temp.renameTo(new File(s));
    }
  }
}
