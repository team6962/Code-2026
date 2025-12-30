package com.team6962.lib.math;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Represents a 2D translational velocity with x and y components. The
 * components are represented by measures, allowing for unit-safe calculations.
 */
public class TranslationalVelocity {
    /**
     * A TranslationalVelocity instance representing zero velocity.
     */
    public static final TranslationalVelocity ZERO = new TranslationalVelocity();
    
    /**
     * The x-component of the translational velocity.
     */
    public final LinearVelocity x;

    /**
     * The y-component of the translational velocity.
     */
    public final LinearVelocity y;

    /**
     * Creates a TranslationalVelocity with the specified x and y components.
     * 
     * @param vx The x-component of the translational velocity
     * @param vy The y-component of the translational velocity
     */
    public TranslationalVelocity(LinearVelocity vx, LinearVelocity vy) {
        this.x = vx;
        this.y = vy;
    }

    /**
     * Creates a TranslationalVelocity from a ChassisSpeeds object. The
     * angular velocity is ignored.
     * 
     * @param speeds The ChassisSpeeds object containing the velocity components
     */
    public TranslationalVelocity(ChassisSpeeds speeds) {
        this.x = MetersPerSecond.of(speeds.vxMetersPerSecond);
        this.y = MetersPerSecond.of(speeds.vyMetersPerSecond);
    }

    /**
     * Creates a TranslationalVelocity from a Translation2d object and a time
     * interval. The translation components are divided by the time interval to
     * obtain the velocity components.
     * 
     * @param translation The Translation2d object representing the translation
     * @param deltaTimeSeconds The time interval in seconds
     */
    public TranslationalVelocity(Translation2d translation, double deltaTimeSeconds) {
        this.x = MetersPerSecond.of(translation.getX() / deltaTimeSeconds);
        this.y = MetersPerSecond.of(translation.getY() / deltaTimeSeconds);
    }

    /**
     * Creates a TranslationalVelocity from a Translation2d object. The
     * components are treated as velocities in meters per second.
     * 
     * @param translationalVelocity The Translation2d object representing the
     *                              translational velocity
     */
    public TranslationalVelocity(Translation2d translationalVelocity) {
        this.x = MetersPerSecond.of(translationalVelocity.getX());
        this.y = MetersPerSecond.of(translationalVelocity.getY());
    }

    /**
     * Creates a TranslationalVelocity with the specified x and y components in
     * meters per second.
     * 
     * @param vxMetersPerSecond The x-component of the translational velocity in
     *                          meters per second
     * @param vyMetersPerSecond The y-component of the translational velocity in
     *                          meters per second
     */
    public TranslationalVelocity(double vxMetersPerSecond, double vyMetersPerSecond) {
        this.x = MetersPerSecond.of(vxMetersPerSecond);
        this.y = MetersPerSecond.of(vyMetersPerSecond);
    }

    /**
     * Creates a TranslationalVelocity from a 2D vector. The first component is
     * the x-component and the second component is the y-component, both in
     * meters per second. Any additional components are ignored.
     * 
     * @param vector The 2D vector representing the translational velocity
     */
    public TranslationalVelocity(Matrix<?, N1> vector) {
        this.x = MetersPerSecond.of(vector.get(0, 0));
        this.y = MetersPerSecond.of(vector.get(1, 0));
    }

    /**
     * Creates a TranslationalVelocity representing zero velocity. Note that
     * this is equivalent to TranslationalVelocity.ZERO, but creates a new
     * instance.
     */
    public TranslationalVelocity() {
        this.x = MetersPerSecond.of(0);
        this.y = MetersPerSecond.of(0);
    }

    /**
     * Calculates the speed of the translational velocity vector.
     * 
     * @return The speed of the translational velocity
     */
    public LinearVelocity getSpeed() {
        return MetersPerSecond.of(Math.hypot(x.in(MetersPerSecond), y.in(MetersPerSecond)));
    }

    /**
     * Calculates the direction of the translational velocity vector as an
     * angle measure.
     * 
     * @return The direction of the translational velocity
     */
    public Angle getDirection() {
        return Radians.of(Math.atan2(y.in(MetersPerSecond), x.in(MetersPerSecond)));
    }

    /**
     * Adds another TranslationalVelocity to this one and returns the result.
     * 
     * @param other The other TranslationalVelocity to add
     * @return The sum of the two TranslationalVelocity instances
     */
    public TranslationalVelocity plus(TranslationalVelocity other) {
        return new TranslationalVelocity(
            MetersPerSecond.of(this.x.in(MetersPerSecond) + other.x.in(MetersPerSecond)),
            MetersPerSecond.of(this.y.in(MetersPerSecond) + other.y.in(MetersPerSecond))
        );
    }

    /**
     * Subtracts another TranslationalVelocity from this one and returns the
     * result.
     * 
     * @param other The other TranslationalVelocity to subtract
     * @return The difference between the two TranslationalVelocity instances
     */
    public TranslationalVelocity minus(TranslationalVelocity other) {
        return new TranslationalVelocity(
            MetersPerSecond.of(this.x.in(MetersPerSecond) - other.x.in(MetersPerSecond)),
            MetersPerSecond.of(this.y.in(MetersPerSecond) - other.y.in(MetersPerSecond))
        );
    }

    /**
     * Multiplies this TranslationalVelocity by a scalar and returns the result.
     * 
     * @param scalar The scalar to multiply by
     * @return The scaled TranslationalVelocity
     */
    public TranslationalVelocity times(double scalar) {
        return new TranslationalVelocity(
            MetersPerSecond.of(this.x.in(MetersPerSecond) * scalar),
            MetersPerSecond.of(this.y.in(MetersPerSecond) * scalar)
        );
    }

    /**
     * Divides this TranslationalVelocity by a scalar and returns the result.
     * 
     * @param scalar The scalar to divide by
     * @return The scaled TranslationalVelocity
     */
    public TranslationalVelocity div(double scalar) {
        return new TranslationalVelocity(
            MetersPerSecond.of(this.x.in(MetersPerSecond) / scalar),
            MetersPerSecond.of(this.y.in(MetersPerSecond) / scalar)
        );
    }

    /**
     * Calculates the unit vector (direction) of this TranslationalVelocity.
     * 
     * @return The unit vector of the TranslationalVelocity
     */
    public TranslationalVelocity unit() {
        LinearVelocity magnitude = getSpeed();
        if (magnitude.in(MetersPerSecond) == 0) {
            return new TranslationalVelocity(
                MetersPerSecond.of(0),
                MetersPerSecond.of(0)
            );
        } else {
            return new TranslationalVelocity(
                MetersPerSecond.of(this.x.in(MetersPerSecond) / magnitude.in(MetersPerSecond)),
                MetersPerSecond.of(this.y.in(MetersPerSecond) / magnitude.in(MetersPerSecond))
            );
        }
    }

    /**
     * Rotates this TranslationalVelocity by the specified angle and returns the
     * result.
     * 
     * @param angle The angle to rotate by
     * @return The rotated TranslationalVelocity
     */
    public TranslationalVelocity rotateBy(Angle angle) {
        double cosAngle = Math.cos(angle.in(Radians));
        double sinAngle = Math.sin(angle.in(Radians));

        double rotatedX = this.x.in(MetersPerSecond) * cosAngle - this.y.in(MetersPerSecond) * sinAngle;
        double rotatedY = this.x.in(MetersPerSecond) * sinAngle + this.y.in(MetersPerSecond) * cosAngle;

        return new TranslationalVelocity(
            MetersPerSecond.of(rotatedX),
            MetersPerSecond.of(rotatedY)
        );
    }

    /**
     * Rotates this TranslationalVelocity by the specified Rotation2d angle
     * and returns the result.
     * 
     * @param angle The Rotation2d angle to rotate by
     * @return The rotated TranslationalVelocity
     */
    public TranslationalVelocity rotateBy(Rotation2d angle) {
        return rotateBy(angle.getMeasure());
    }

    /**
     * Converts this TranslationalVelocity to a 2D vector representation.
     * 
     * @return The 2D vector representing the TranslationalVelocity
     */
    public Vector<N2> toVector() {
        Vector<N2> vector = new Vector<N2>(Nat.N2());

        vector.set(0, 0, x.in(MetersPerSecond));
        vector.set(1, 0, y.in(MetersPerSecond));

        return vector;
    }

    @Override
    public String toString() {
        return String.format("TranslationalVelocity(Vx: %.2f, Vy: %.2f)", x.in(MetersPerSecond), y.in(MetersPerSecond));
    }
}
