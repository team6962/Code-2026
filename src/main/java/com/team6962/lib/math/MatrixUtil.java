package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;

public class MatrixUtil {
    public static Vector<N3> toVector3(Vector<N2> input) {
        Vector<N3> output = new Vector<N3>(Nat.N3());

        output.set(0, 0, input.get(0, 0));
        output.set(1, 0, input.get(1, 0));
        output.set(2, 0, 1.0);

        return output;
    }

    public static Vector<N2> toVector2(Vector<N3> input) {
        Vector<N2> output = new Vector<N2>(Nat.N2());

        output.set(0, 0, input.get(0, 0));
        output.set(1, 0, input.get(1, 0));

        return output;
    }

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

    public static Matrix<N3, N3> toMatrix3x3(Matrix<N2, N2> input) {
        Matrix<N3, N3> output = Matrix.eye(Nat.N3());

        output.set(0, 0, input.get(0, 0));
        output.set(0, 1, input.get(0, 1));
        output.set(1, 0, input.get(1, 0));
        output.set(1, 1, input.get(1, 1));

        return output;
    }

    public static Matrix<N2, N2> toMatrix2x2(Matrix<N3, N3> input) {
        Matrix<N2, N2> output = Matrix.eye(Nat.N2());

        output.set(0, 0, input.get(0, 0));
        output.set(0, 1, input.get(0, 1));
        output.set(1, 0, input.get(1, 0));
        output.set(1, 1, input.get(1, 1));

        return output;
    }

    public static Matrix<N3, N3> createTranslation3x3(Vector<?> translation) {
        Matrix<N3, N3> translationMatrix = Matrix.eye(Nat.N3());

        translationMatrix.set(0, 2, translation.get(0));
        translationMatrix.set(1, 2, translation.get(1));

        return translationMatrix;
    }

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
