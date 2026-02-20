package com.team6962.lib.math;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import org.apache.commons.io.FileUtils;

public class CSVLoader {

  /**
   * Loads a CSV file containing floating-point values into a two-dimensional double array.
   *
   * <p>The provided path is resolved relative to the deploy directory:
   * Filesystem.getDeployDirectory() + "/" + path. The file is read using UTF-8 encoding. The file
   * contents are split into rows on the newline character ('\n'), and each row is split into
   * columns on the comma (',') character. Each column token is parsed with Double.parseDouble and
   * stored in the returned double[][]. The resulting array has one entry per line; rows may be
   * ragged (different numbers of columns per line).
   *
   * @param path the path to the CSV file relative to the deploy directory
   * @return a two-dimensional array of doubles where each outer element represents a CSV line and
   *     each inner element represents a parsed numeric value from that line
   * @throws IOException if an I/O error occurs while reading the file
   * @throws NumberFormatException if any token cannot be parsed as a double
   */
  public static double[][] loadCSV(String path) throws IOException {
    path = Filesystem.getDeployDirectory() + "/" + path;
    String csv =
        FileUtils.readFileToString(new File(path), "UTF-8"); // Read the CSV file as a string
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
