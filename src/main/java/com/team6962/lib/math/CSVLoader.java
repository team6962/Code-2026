package com.team6962.lib.math;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import org.apache.commons.io.FileUtils;

public class CSVLoader {
  public static double[][] loadCSV(String path) throws IOException {
    path = Filesystem.getDeployDirectory() + "/" + path;
    String csv = FileUtils.readFileToString(new File(path), "UTF-8"); // Read the CSV file as a string
    String[] lines = csv.split("\n"); // Splits the strings along lines
    double[][] data = new double[lines.length][];
    for (int i = 0; i < lines.length; i++) {
      String[] values = lines[i].split(","); // Splits each line along commas
      data[i] = new double[values.length];
      for (int j = 0; j < values.length; j++) {
        data[i][j] = Double.parseDouble(values[j]);
      }
    }
    return data;
  }
}
