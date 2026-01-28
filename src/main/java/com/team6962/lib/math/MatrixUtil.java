package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;

/** Utility methods for working with matrices and vectors. */
public class MatrixUtil {
  /**
   * Converts a 2D vector to a 3D vector with the components (x, y, 1). This allows the vector to be
   * translated using a 3x3 transformation matrix.
   *
   * @param input The 2D vector to convert
   * @return The equivalent 3D vector with components (x, y, 1)
   */
  public static Vector<N3> toVector3(Vector<N2> input) {
    Vector<N3> output = new Vector<N3>(Nat.N3());

    output.set(0, 0, input.get(0, 0));
    output.set(1, 0, input.get(1, 0));
    output.set(2, 0, 1.0);

    return output;
  }

  /**
   * Converts a 3D vector to a 2D vector by dropping the third component.
   *
   * @param input The 3D vector to convert
   * @return The equivalent 2D vector with the third component dropped
   */
  public static Vector<N2> toVector2(Vector<N3> input) {
    Vector<N2> output = new Vector<N2>(Nat.N2());

    output.set(0, 0, input.get(0, 0));
    output.set(1, 0, input.get(1, 0));

    return output;
  }

  /**
   * Creates a 2x2 rotation matrix that rotates vectors counterclockwise around the origin by the
   * specified angle.
   *
   * @param angle The angle to rotate by
   * @return The 2x2 rotation matrix
   */
  public static Matrix<N2, N2> createRotation2x2(Angle angle) {
    double cosAngle = Math.cos(angle.in(Radians));
    double sinAngle = Math.sin(angle.in(Radians));

    Matrix<N2, N2> rotationMatrix = Matrix.eye(Nat.N2());

    rotationMatrix.set(0, 0, cosAngle);
    rotationMatrix.set(0, 1, -sinAngle);
    rotationMatrix.set(1, 0, sinAngle);
    rotationMatrix.set(1, 1, cosAngle);

    return rotationMatrix;
  }

  /**
   * Creates a 2x2 rotation matrix from a direction vector, where the first component is the cosine
   * of the angle and the second component is the sine of the angle. If the magnitude of the
   * direction vector is not 1, the resulting matrix will also scale vectors by that magnitude.
   *
   * @param direction The direction vector containing the cosine and sine of the angle
   * @return The 2x2 rotation matrix
   */
  public static Matrix<N2, N2> createRotation2x2(Vector<?> direction) {
    double cosAngle = direction.get(0, 0);
    double sinAngle = direction.get(1, 0);

    Matrix<N2, N2> rotationMatrix = Matrix.eye(Nat.N2());

    rotationMatrix.set(0, 0, cosAngle);
    rotationMatrix.set(0, 1, -sinAngle);
    rotationMatrix.set(1, 0, sinAngle);
    rotationMatrix.set(1, 1, cosAngle);

    return rotationMatrix;
  }

  /**
   * Converts a 2x2 matrix to a 3x3 matrix by embedding it in the top-left corner and setting the
   * bottom-right element to 1. The resulting matrix is capable of translating 2D vectors, unlike
   * the original 2x2 matrix.
   *
   * @param input The 2x2 matrix to convert
   * @return The equivalent 3x3 matrix
   */
  public static Matrix<N3, N3> toMatrix3x3(Matrix<N2, N2> input) {
    Matrix<N3, N3> output = Matrix.eye(Nat.N3());

    output.set(0, 0, input.get(0, 0));
    output.set(0, 1, input.get(0, 1));
    output.set(1, 0, input.get(1, 0));
    output.set(1, 1, input.get(1, 1));

    return output;
  }

  /**
   * Converts a 3x3 matrix to a 2x2 matrix by extracting the top-left 2x2 submatrix. The resulting
   * matrix does not translate vectors, unlike the original 3x3 matrix.
   *
   * @param input The 3x3 matrix to convert
   * @return The equivalent 2x2 matrix
   */
  public static Matrix<N2, N2> toMatrix2x2(Matrix<N3, N3> input) {
    Matrix<N2, N2> output = Matrix.eye(Nat.N2());

    output.set(0, 0, input.get(0, 0));
    output.set(0, 1, input.get(0, 1));
    output.set(1, 0, input.get(1, 0));
    output.set(1, 1, input.get(1, 1));

    return output;
  }

  /**
   * Creates a 3x3 translation matrix that translates vectors by the specified 2D translation
   * vector.
   *
   * @param translation The 2D translation vector
   * @return The 3x3 translation matrix
   */
  public static Matrix<N3, N3> createTranslation3x3(Vector<?> translation) {
    Matrix<N3, N3> translationMatrix = Matrix.eye(Nat.N3());

    translationMatrix.set(0, 2, translation.get(0));
    translationMatrix.set(1, 2, translation.get(1));

    return translationMatrix;
  }

  /**
   * Creates a 3x3 rotation matrix that rotates vectors counterclockwise around the origin by the
   * specified angle.
   *
   * @param angle The angle to rotate by
   * @return The 3x3 rotation matrix
   */
  public static Matrix<N3, N3> createRotation3x3(Angle angle) {
    double cosAngle = Math.cos(angle.in(Radians));
    double sinAngle = Math.sin(angle.in(Radians));

    Matrix<N3, N3> rotationMatrix = Matrix.eye(Nat.N3());

    rotationMatrix.set(0, 0, cosAngle);
    rotationMatrix.set(0, 1, -sinAngle);
    rotationMatrix.set(1, 0, sinAngle);
    rotationMatrix.set(1, 1, cosAngle);

    return rotationMatrix;
  }

  /**
   * Creates a 3x3 rotation matrix from a direction vector, where the first component is the cosine
   * of the angle and the second component is the sine of the angle. If the magnitude of the
   * direction vector is not 1, the resulting matrix will also scale vectors by that magnitude.
   *
   * @param direction The direction vector containing the cosine and sine of the angle
   * @return The 3x3 rotation matrix
   */
  public static Matrix<N3, N3> createRotation3x3(Vector<?> direction) {
    double cosAngle = direction.get(0, 0);
    double sinAngle = direction.get(1, 0);

    Matrix<N3, N3> rotationMatrix = Matrix.eye(Nat.N3());

    rotationMatrix.set(0, 0, cosAngle);
    rotationMatrix.set(0, 1, -sinAngle);
    rotationMatrix.set(1, 0, sinAngle);
    rotationMatrix.set(1, 1, cosAngle);

    return rotationMatrix;
  }
}
