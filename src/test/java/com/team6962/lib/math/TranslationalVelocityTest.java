package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.LinearVelocity;

class TranslationalVelocityTest {

    private static final double DELTA = 1e-9;

    @Test
    void constructor_LinearVelocities_SetsComponents() {
        LinearVelocity vx = MetersPerSecond.of(3.0);
        LinearVelocity vy = MetersPerSecond.of(4.0);

        TranslationalVelocity velocity = new TranslationalVelocity(vx, vy);

        assertEquals(3.0, velocity.x.in(MetersPerSecond), DELTA);
        assertEquals(4.0, velocity.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void constructor_ChassisSpeeds_ExtractsComponents() {
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 2.0, 0.5);

        TranslationalVelocity velocity = new TranslationalVelocity(speeds);

        assertEquals(1.0, velocity.x.in(MetersPerSecond), DELTA);
        assertEquals(2.0, velocity.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void constructor_Translation2dWithTime_CalculatesVelocity() {
        Translation2d translation = new Translation2d(6.0, 8.0);
        double deltaTime = 2.0;

        TranslationalVelocity velocity = new TranslationalVelocity(translation, deltaTime);

        assertEquals(3.0, velocity.x.in(MetersPerSecond), DELTA);
        assertEquals(4.0, velocity.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void constructor_Translation2d_TreatsAsVelocity() {
        Translation2d translationalVelocity = new Translation2d(5.0, 7.0);

        TranslationalVelocity velocity = new TranslationalVelocity(translationalVelocity);

        assertEquals(5.0, velocity.x.in(MetersPerSecond), DELTA);
        assertEquals(7.0, velocity.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void constructor_Doubles_SetsComponents() {
        TranslationalVelocity velocity = new TranslationalVelocity(3.0, 4.0);

        assertEquals(3.0, velocity.x.in(MetersPerSecond), DELTA);
        assertEquals(4.0, velocity.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void constructor_Vector_SetsComponents() {
        Vector<N2> vector = new Vector<>(Nat.N2());
        vector.set(0, 0, 5.0);
        vector.set(1, 0, 6.0);

        TranslationalVelocity velocity = new TranslationalVelocity(vector);

        assertEquals(5.0, velocity.x.in(MetersPerSecond), DELTA);
        assertEquals(6.0, velocity.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void constructor_Default_ZeroVelocity() {
        TranslationalVelocity velocity = new TranslationalVelocity();

        assertEquals(0.0, velocity.x.in(MetersPerSecond), DELTA);
        assertEquals(0.0, velocity.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void zero_Constant_HasZeroComponents() {
        assertEquals(0.0, TranslationalVelocity.ZERO.x.in(MetersPerSecond), DELTA);
        assertEquals(0.0, TranslationalVelocity.ZERO.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void getSpeed_345Triangle_Returns5() {
        TranslationalVelocity velocity = new TranslationalVelocity(3.0, 4.0);

        assertEquals(5.0, velocity.getSpeed().in(MetersPerSecond), DELTA);
    }

    @Test
    void getSpeed_ZeroVelocity_ReturnsZero() {
        TranslationalVelocity velocity = new TranslationalVelocity(0.0, 0.0);

        assertEquals(0.0, velocity.getSpeed().in(MetersPerSecond), DELTA);
    }

    @Test
    void getDirection_PositiveX_ReturnsZero() {
        TranslationalVelocity velocity = new TranslationalVelocity(1.0, 0.0);

        assertEquals(0.0, velocity.getDirection().in(Radians), DELTA);
    }

    @Test
    void getDirection_PositiveY_Returns90Degrees() {
        TranslationalVelocity velocity = new TranslationalVelocity(0.0, 1.0);

        assertEquals(90.0, velocity.getDirection().in(Degrees), DELTA);
    }

    @Test
    void getDirection_NegativeX_Returns180Degrees() {
        TranslationalVelocity velocity = new TranslationalVelocity(-1.0, 0.0);

        assertEquals(180.0, Math.abs(velocity.getDirection().in(Degrees)), DELTA);
    }

    @Test
    void getDirection_45Degrees() {
        TranslationalVelocity velocity = new TranslationalVelocity(1.0, 1.0);

        assertEquals(45.0, velocity.getDirection().in(Degrees), DELTA);
    }

    @Test
    void plus_TwoVelocities_AddsComponents() {
        TranslationalVelocity a = new TranslationalVelocity(1.0, 2.0);
        TranslationalVelocity b = new TranslationalVelocity(3.0, 4.0);

        TranslationalVelocity result = a.plus(b);

        assertEquals(4.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(6.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void minus_TwoVelocities_SubtractsComponents() {
        TranslationalVelocity a = new TranslationalVelocity(5.0, 7.0);
        TranslationalVelocity b = new TranslationalVelocity(2.0, 3.0);

        TranslationalVelocity result = a.minus(b);

        assertEquals(3.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(4.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void times_Scalar_MultipliesComponents() {
        TranslationalVelocity velocity = new TranslationalVelocity(2.0, 3.0);

        TranslationalVelocity result = velocity.times(2.0);

        assertEquals(4.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(6.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void times_NegativeScalar_NegatesComponents() {
        TranslationalVelocity velocity = new TranslationalVelocity(2.0, 3.0);

        TranslationalVelocity result = velocity.times(-1.0);

        assertEquals(-2.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(-3.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void div_Scalar_DividesComponents() {
        TranslationalVelocity velocity = new TranslationalVelocity(6.0, 8.0);

        TranslationalVelocity result = velocity.div(2.0);

        assertEquals(3.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(4.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void unit_NonZeroVelocity_ReturnsMagnitudeOne() {
        TranslationalVelocity velocity = new TranslationalVelocity(3.0, 4.0);

        TranslationalVelocity result = velocity.unit();

        assertEquals(1.0, result.getSpeed().in(MetersPerSecond), DELTA);
        assertEquals(0.6, result.x.in(MetersPerSecond), DELTA);
        assertEquals(0.8, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void unit_ZeroVelocity_ReturnsZero() {
        TranslationalVelocity velocity = new TranslationalVelocity(0.0, 0.0);

        TranslationalVelocity result = velocity.unit();

        assertEquals(0.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(0.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void rotateBy_Angle_90Degrees_RotatesCorrectly() {
        TranslationalVelocity velocity = new TranslationalVelocity(1.0, 0.0);

        TranslationalVelocity result = velocity.rotateBy(Degrees.of(90));

        assertEquals(0.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(1.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void rotateBy_Angle_180Degrees_NegatesX() {
        TranslationalVelocity velocity = new TranslationalVelocity(1.0, 0.0);

        TranslationalVelocity result = velocity.rotateBy(Degrees.of(180));

        assertEquals(-1.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(0.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void rotateBy_Rotation2d_90Degrees_RotatesCorrectly() {
        TranslationalVelocity velocity = new TranslationalVelocity(1.0, 0.0);

        TranslationalVelocity result = velocity.rotateBy(Rotation2d.fromDegrees(90));

        assertEquals(0.0, result.x.in(MetersPerSecond), DELTA);
        assertEquals(1.0, result.y.in(MetersPerSecond), DELTA);
    }

    @Test
    void toVector_ReturnsCorrectVector() {
        TranslationalVelocity velocity = new TranslationalVelocity(3.0, 4.0);

        Vector<N2> result = velocity.toVector();

        assertEquals(3.0, result.get(0, 0), DELTA);
        assertEquals(4.0, result.get(1, 0), DELTA);
    }

    @Test
    void toString_FormatsCorrectly() {
        TranslationalVelocity velocity = new TranslationalVelocity(1.5, 2.5);

        String result = velocity.toString();

        assertEquals("TranslationalVelocity(Vx: 1.50, Vy: 2.50)", result);
    }
}
