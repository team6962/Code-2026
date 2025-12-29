package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.team6962.lib.math.TranslationalVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Frequency;

public class TranslationController {
    private TrapezoidalController xController;
    private TrapezoidalController yController;
    private Translation2d goalPosition;

    public TranslationController(
        double kP, double kI, double kD,
        TrapezoidProfile.Constraints constraints,
        Frequency updateFrequency
    ) {
        this.xController = new TrapezoidalController(kP, kI, kD, constraints, updateFrequency);
        this.yController = new TrapezoidalController(kP, kI, kD, constraints, updateFrequency);
    }

    public void setProfile(Translation2d initialPosition,TranslationalVelocity initialVelocity, Translation2d goalTranslation, TranslationalVelocity goalVelocity) {
        this.goalPosition = goalTranslation;

        xController.setProfile(
            new TrapezoidProfile.State(initialPosition.getX(), initialVelocity.x.in(MetersPerSecond)),
            new TrapezoidProfile.State(goalPosition.getX(), goalVelocity.x.in(MetersPerSecond))
        );

        yController.setProfile(
            new TrapezoidProfile.State(initialPosition.getY(), initialVelocity.y.in(MetersPerSecond)),
            new TrapezoidProfile.State(goalPosition.getY(), goalVelocity.y.in(MetersPerSecond))
        );
        
        // Synchronize durations
        setDuration(getDuration());
    }

    public double getDuration() {
        return Math.max(xController.getDuration(), yController.getDuration());
    }

    public void setDuration(double duration) {
        xController.setDuration(duration);
        yController.setDuration(duration);
    }
    
    public TranslationalVelocity calculate(Translation2d position, TranslationalVelocity velocity) {
        Pose2d goalPose = new Pose2d(goalPosition, new Rotation2d());
        Pose2d currentPose = new Pose2d(position, new Rotation2d());
        Pose2d currentPoseRelativeToGoal = currentPose.relativeTo(goalPose);

        TranslationalVelocity currentVelocity = new TranslationalVelocity(
            velocity.x.in(MetersPerSecond),
            velocity.y.in(MetersPerSecond)
        ).rotateBy(currentPoseRelativeToGoal.getRotation().unaryMinus());

        double xVelocity = xController.calculate(new TrapezoidProfile.State(
            Meters.of(currentPoseRelativeToGoal.getX()).in(Meters),
            currentVelocity.x.in(MetersPerSecond)
        ));

        double yVelocity = yController.calculate(new TrapezoidProfile.State(
            Meters.of(currentPoseRelativeToGoal.getY()).in(Meters),
            currentVelocity.y.in(MetersPerSecond)
        ));

        return new TranslationalVelocity(xVelocity, yVelocity)
            .rotateBy(currentPoseRelativeToGoal.getRotation());
    }

    public boolean isFinished() {
        return xController.isFinished() && yController.isFinished();
    }
}
