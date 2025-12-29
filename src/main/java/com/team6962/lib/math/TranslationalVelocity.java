package com.team6962.lib.math;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class TranslationalVelocity {
    public static final TranslationalVelocity ZERO = new TranslationalVelocity();
    
    public final LinearVelocity x;
    public final LinearVelocity y;

    public TranslationalVelocity(LinearVelocity vx, LinearVelocity vy) {
        this.x = vx;
        this.y = vy;
    }

    public TranslationalVelocity(ChassisSpeeds speeds) {
        this.x = MetersPerSecond.of(speeds.vxMetersPerSecond);
        this.y = MetersPerSecond.of(speeds.vyMetersPerSecond);
    }

    public TranslationalVelocity(Translation2d translation, double deltaTimeSeconds) {
        this.x = MetersPerSecond.of(translation.getX() / deltaTimeSeconds);
        this.y = MetersPerSecond.of(translation.getY() / deltaTimeSeconds);
    }

    public TranslationalVelocity(Translation2d translationalVelocity) {
        this.x = MetersPerSecond.of(translationalVelocity.getX());
        this.y = MetersPerSecond.of(translationalVelocity.getY());
    }

    public TranslationalVelocity(double vxMetersPerSecond, double vyMetersPerSecond) {
        this.x = MetersPerSecond.of(vxMetersPerSecond);
        this.y = MetersPerSecond.of(vyMetersPerSecond);
    }

    public TranslationalVelocity() {
        this.x = MetersPerSecond.of(0);
        this.y = MetersPerSecond.of(0);
    }

    public LinearVelocity getMagnitude() {
        return MetersPerSecond.of(Math.hypot(x.in(MetersPerSecond), y.in(MetersPerSecond)));
    }

    public Angle getDirection() {
        return Radians.of(Math.atan2(y.in(MetersPerSecond), x.in(MetersPerSecond)));
    }

    public TranslationalVelocity plus(TranslationalVelocity other) {
        return new TranslationalVelocity(
            MetersPerSecond.of(this.x.in(MetersPerSecond) + other.x.in(MetersPerSecond)),
            MetersPerSecond.of(this.y.in(MetersPerSecond) + other.y.in(MetersPerSecond))
        );
    }

    public TranslationalVelocity minus(TranslationalVelocity other) {
        return new TranslationalVelocity(
            MetersPerSecond.of(this.x.in(MetersPerSecond) - other.x.in(MetersPerSecond)),
            MetersPerSecond.of(this.y.in(MetersPerSecond) - other.y.in(MetersPerSecond))
        );
    }

    public TranslationalVelocity times(double scalar) {
        return new TranslationalVelocity(
            MetersPerSecond.of(this.x.in(MetersPerSecond) * scalar),
            MetersPerSecond.of(this.y.in(MetersPerSecond) * scalar)
        );
    }

    public TranslationalVelocity div(double scalar) {
        return new TranslationalVelocity(
            MetersPerSecond.of(this.x.in(MetersPerSecond) / scalar),
            MetersPerSecond.of(this.y.in(MetersPerSecond) / scalar)
        );
    }

    public TranslationalVelocity unit() {
        LinearVelocity magnitude = getMagnitude();
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

    public TranslationalVelocity rotateBy(Rotation2d angle) {
        return rotateBy(angle.getMeasure());
    }

    @Override
    public String toString() {
        return "TranslationalVelocity(vx=" + x + ", vy=" + y + ")";
    }
}
