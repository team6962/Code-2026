package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import org.junit.jupiter.api.Test;

class MatrixUtilTest {

  private static final double DELTA = 1e-9;

  // ==================== toVector3() tests ====================

  @Test
  void toVector3_BasicVector_ReturnsCorrect3DVector() {
    Vector<N2> input = new Vector<>(Nat.N2());
    input.set(0, 0, 3.0);
    input.set(1, 0, 4.0);

    Vector<N3> result = MatrixUtil.toVector3(input);

    assertEquals(3.0, result.get(0, 0), DELTA);
    assertEquals(4.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(2, 0), DELTA);
  }

  @Test
  void toVector3_ZeroVector_ReturnsZeroWithOneInThirdComponent() {
    Vector<N2> input = new Vector<>(Nat.N2());
    input.set(0, 0, 0.0);
    input.set(1, 0, 0.0);

    Vector<N3> result = MatrixUtil.toVector3(input);

    assertEquals(0.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(2, 0), DELTA);
  }

  @Test
  void toVector3_NegativeValues_PreservesSign() {
    Vector<N2> input = new Vector<>(Nat.N2());
    input.set(0, 0, -5.0);
    input.set(1, 0, -7.0);

    Vector<N3> result = MatrixUtil.toVector3(input);

    assertEquals(-5.0, result.get(0, 0), DELTA);
    assertEquals(-7.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(2, 0), DELTA);
  }

  // ==================== toVector2() tests ====================

  @Test
  void toVector2_BasicVector_DropsThirdComponent() {
    Vector<N3> input = new Vector<>(Nat.N3());
    input.set(0, 0, 3.0);
    input.set(1, 0, 4.0);
    input.set(2, 0, 99.0);

    Vector<N2> result = MatrixUtil.toVector2(input);

    assertEquals(3.0, result.get(0, 0), DELTA);
    assertEquals(4.0, result.get(1, 0), DELTA);
  }

  @Test
  void toVector2_RoundTrip_PreservesOriginal() {
    Vector<N2> original = new Vector<>(Nat.N2());
    original.set(0, 0, 7.0);
    original.set(1, 0, 11.0);

    Vector<N3> intermediate = MatrixUtil.toVector3(original);
    Vector<N2> result = MatrixUtil.toVector2(intermediate);

    assertEquals(original.get(0, 0), result.get(0, 0), DELTA);
    assertEquals(original.get(1, 0), result.get(1, 0), DELTA);
  }

  // ==================== createRotation2x2(Angle) tests ====================

  @Test
  void createRotation2x2_ZeroAngle_ReturnsIdentity() {
    Matrix<N2, N2> result = MatrixUtil.createRotation2x2(Radians.of(0));

    assertEquals(1.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(0, 1), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(1, 1), DELTA);
  }

  @Test
  void createRotation2x2_90Degrees_RotatesCorrectly() {
    Matrix<N2, N2> rotation = MatrixUtil.createRotation2x2(Degrees.of(90));

    // cos(90) = 0, sin(90) = 1
    assertEquals(0.0, rotation.get(0, 0), DELTA);
    assertEquals(-1.0, rotation.get(0, 1), DELTA);
    assertEquals(1.0, rotation.get(1, 0), DELTA);
    assertEquals(0.0, rotation.get(1, 1), DELTA);
  }

  @Test
  void createRotation2x2_45Degrees_RotatesCorrectly() {
    Matrix<N2, N2> rotation = MatrixUtil.createRotation2x2(Degrees.of(45));

    double expected = Math.sqrt(2) / 2;
    assertEquals(expected, rotation.get(0, 0), DELTA);
    assertEquals(-expected, rotation.get(0, 1), DELTA);
    assertEquals(expected, rotation.get(1, 0), DELTA);
    assertEquals(expected, rotation.get(1, 1), DELTA);
  }

  @Test
  void createRotation2x2_AppliedToVector_RotatesVector() {
    // Rotate (1, 0) by 90 degrees should give (0, 1)
    Matrix<N2, N2> rotation = MatrixUtil.createRotation2x2(Degrees.of(90));
    Vector<N2> vector = new Vector<>(Nat.N2());
    vector.set(0, 0, 1.0);
    vector.set(1, 0, 0.0);

    Matrix<N2, ?> result = rotation.times(vector);

    assertEquals(0.0, result.get(0, 0), DELTA);
    assertEquals(1.0, result.get(1, 0), DELTA);
  }

  // ==================== createRotation2x2(Vector) tests ====================

  @Test
  void createRotation2x2_FromUnitVector_0Degrees() {
    // cos(0) = 1, sin(0) = 0
    Vector<N2> direction = new Vector<>(Nat.N2());
    direction.set(0, 0, 1.0);
    direction.set(1, 0, 0.0);

    Matrix<N2, N2> result = MatrixUtil.createRotation2x2(direction);

    assertEquals(1.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(0, 1), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(1, 1), DELTA);
  }

  @Test
  void createRotation2x2_FromUnitVector_90Degrees() {
    // cos(90) = 0, sin(90) = 1
    Vector<N2> direction = new Vector<>(Nat.N2());
    direction.set(0, 0, 0.0);
    direction.set(1, 0, 1.0);

    Matrix<N2, N2> result = MatrixUtil.createRotation2x2(direction);

    assertEquals(0.0, result.get(0, 0), DELTA);
    assertEquals(-1.0, result.get(0, 1), DELTA);
    assertEquals(1.0, result.get(1, 0), DELTA);
    assertEquals(0.0, result.get(1, 1), DELTA);
  }

  // ==================== toMatrix3x3() tests ====================

  @Test
  void toMatrix3x3_IdentityMatrix_Returns3x3Identity() {
    Matrix<N2, N2> input = Matrix.eye(Nat.N2());

    Matrix<N3, N3> result = MatrixUtil.toMatrix3x3(input);

    assertEquals(1.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(0, 1), DELTA);
    assertEquals(0.0, result.get(0, 2), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(1, 1), DELTA);
    assertEquals(0.0, result.get(1, 2), DELTA);
    assertEquals(0.0, result.get(2, 0), DELTA);
    assertEquals(0.0, result.get(2, 1), DELTA);
    assertEquals(1.0, result.get(2, 2), DELTA);
  }

  @Test
  void toMatrix3x3_CustomMatrix_EmbedsCorrectly() {
    Matrix<N2, N2> input = Matrix.eye(Nat.N2());
    input.set(0, 0, 2.0);
    input.set(0, 1, 3.0);
    input.set(1, 0, 4.0);
    input.set(1, 1, 5.0);

    Matrix<N3, N3> result = MatrixUtil.toMatrix3x3(input);

    assertEquals(2.0, result.get(0, 0), DELTA);
    assertEquals(3.0, result.get(0, 1), DELTA);
    assertEquals(0.0, result.get(0, 2), DELTA);
    assertEquals(4.0, result.get(1, 0), DELTA);
    assertEquals(5.0, result.get(1, 1), DELTA);
    assertEquals(0.0, result.get(1, 2), DELTA);
    assertEquals(0.0, result.get(2, 0), DELTA);
    assertEquals(0.0, result.get(2, 1), DELTA);
    assertEquals(1.0, result.get(2, 2), DELTA);
  }

  // ==================== toMatrix2x2() tests ====================

  @Test
  void toMatrix2x2_Identity3x3_Returns2x2Identity() {
    Matrix<N3, N3> input = Matrix.eye(Nat.N3());

    Matrix<N2, N2> result = MatrixUtil.toMatrix2x2(input);

    assertEquals(1.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(0, 1), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(1, 1), DELTA);
  }

  @Test
  void toMatrix2x2_RoundTrip_PreservesOriginal() {
    Matrix<N2, N2> original = Matrix.eye(Nat.N2());
    original.set(0, 0, 7.0);
    original.set(0, 1, 8.0);
    original.set(1, 0, 9.0);
    original.set(1, 1, 10.0);

    Matrix<N3, N3> intermediate = MatrixUtil.toMatrix3x3(original);
    Matrix<N2, N2> result = MatrixUtil.toMatrix2x2(intermediate);

    assertEquals(original.get(0, 0), result.get(0, 0), DELTA);
    assertEquals(original.get(0, 1), result.get(0, 1), DELTA);
    assertEquals(original.get(1, 0), result.get(1, 0), DELTA);
    assertEquals(original.get(1, 1), result.get(1, 1), DELTA);
  }

  // ==================== createTranslation3x3() tests ====================

  @Test
  void createTranslation3x3_ZeroTranslation_ReturnsIdentity() {
    Vector<N2> translation = new Vector<>(Nat.N2());
    translation.set(0, 0, 0.0);
    translation.set(1, 0, 0.0);

    Matrix<N3, N3> result = MatrixUtil.createTranslation3x3(translation);

    assertEquals(1.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(0, 1), DELTA);
    assertEquals(0.0, result.get(0, 2), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(1, 1), DELTA);
    assertEquals(0.0, result.get(1, 2), DELTA);
    assertEquals(0.0, result.get(2, 0), DELTA);
    assertEquals(0.0, result.get(2, 1), DELTA);
    assertEquals(1.0, result.get(2, 2), DELTA);
  }

  @Test
  void createTranslation3x3_NonZeroTranslation_SetsCorrectElements() {
    Vector<N2> translation = new Vector<>(Nat.N2());
    translation.set(0, 0, 5.0);
    translation.set(1, 0, 7.0);

    Matrix<N3, N3> result = MatrixUtil.createTranslation3x3(translation);

    assertEquals(1.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(0, 1), DELTA);
    assertEquals(5.0, result.get(0, 2), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(1, 1), DELTA);
    assertEquals(7.0, result.get(1, 2), DELTA);
    assertEquals(0.0, result.get(2, 0), DELTA);
    assertEquals(0.0, result.get(2, 1), DELTA);
    assertEquals(1.0, result.get(2, 2), DELTA);
  }

  @Test
  void createTranslation3x3_AppliedToVector_TranslatesVector() {
    Vector<N2> translation = new Vector<>(Nat.N2());
    translation.set(0, 0, 3.0);
    translation.set(1, 0, 4.0);

    Matrix<N3, N3> translationMatrix = MatrixUtil.createTranslation3x3(translation);

    // Create a point at (1, 2)
    Vector<N3> point = MatrixUtil.toVector3(new Vector<>(Nat.N2()));
    point.set(0, 0, 1.0);
    point.set(1, 0, 2.0);
    point.set(2, 0, 1.0);

    Matrix<N3, ?> result = translationMatrix.times(point);

    assertEquals(4.0, result.get(0, 0), DELTA); // 1 + 3
    assertEquals(6.0, result.get(1, 0), DELTA); // 2 + 4
    assertEquals(1.0, result.get(2, 0), DELTA);
  }

  // ==================== createRotation3x3(Angle) tests ====================

  @Test
  void createRotation3x3_ZeroAngle_ReturnsIdentity() {
    Matrix<N3, N3> result = MatrixUtil.createRotation3x3(Radians.of(0));

    assertEquals(1.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(0, 1), DELTA);
    assertEquals(0.0, result.get(0, 2), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(1, 1), DELTA);
    assertEquals(0.0, result.get(1, 2), DELTA);
    assertEquals(0.0, result.get(2, 0), DELTA);
    assertEquals(0.0, result.get(2, 1), DELTA);
    assertEquals(1.0, result.get(2, 2), DELTA);
  }

  @Test
  void createRotation3x3_90Degrees_RotatesCorrectly() {
    Matrix<N3, N3> rotation = MatrixUtil.createRotation3x3(Degrees.of(90));

    assertEquals(0.0, rotation.get(0, 0), DELTA);
    assertEquals(-1.0, rotation.get(0, 1), DELTA);
    assertEquals(0.0, rotation.get(0, 2), DELTA);
    assertEquals(1.0, rotation.get(1, 0), DELTA);
    assertEquals(0.0, rotation.get(1, 1), DELTA);
    assertEquals(0.0, rotation.get(1, 2), DELTA);
    assertEquals(0.0, rotation.get(2, 0), DELTA);
    assertEquals(0.0, rotation.get(2, 1), DELTA);
    assertEquals(1.0, rotation.get(2, 2), DELTA);
  }

  // ==================== createRotation3x3(Vector) tests ====================

  @Test
  void createRotation3x3_FromUnitVector_0Degrees() {
    Vector<N2> direction = new Vector<>(Nat.N2());
    direction.set(0, 0, 1.0);
    direction.set(1, 0, 0.0);

    Matrix<N3, N3> result = MatrixUtil.createRotation3x3(direction);

    assertEquals(1.0, result.get(0, 0), DELTA);
    assertEquals(0.0, result.get(0, 1), DELTA);
    assertEquals(0.0, result.get(1, 0), DELTA);
    assertEquals(1.0, result.get(1, 1), DELTA);
    assertEquals(1.0, result.get(2, 2), DELTA);
  }
}
