package com.team6962.lib.math;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class TranslationalVelocity {
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

    @Override
    public String toString() {
        return "TranslationalVelocity(vx=" + x + ", vy=" + y + ")";
    }
}
