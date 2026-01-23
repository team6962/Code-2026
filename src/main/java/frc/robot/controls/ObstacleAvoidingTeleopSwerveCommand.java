package frc.robot.controls;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.team6962.lib.control.SimpleTrapezoidalController;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import com.team6962.lib.swerve.config.DrivingConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ObstacleAvoidingTeleopSwerveCommand extends XBoxTeleopSwerveCommand {
    private static double FIELD_LENGTH = Units.inchesToMeters(651);
    private static double FIELD_WIDTH = Units.inchesToMeters(315);
    private static double BARRIER_START = 3.446;
    private static double BARRIER_END = 5.832;
    private static double BARRIER_MIDDLE = (BARRIER_START + BARRIER_END) / 2.0;
    private static double TRENCH_BUMP_BOUNDARY = 1.354;
    private static double TRENCH_CENTER = 0.636;
    private static double BUMP_START = 2.192;
    private static double BUMP_END = 2.818;

    private SimpleTrapezoidalController controller;

    public ObstacleAvoidingTeleopSwerveCommand(
        CommandSwerveDrive swerveDrive,
        XBoxTeleopSwerveConstants constants
    ) {
        super(swerveDrive, constants);

        DrivingConstants drivingConstants = swerveDrive.getConstants().Driving;

        controller = new SimpleTrapezoidalController(
            drivingConstants.TranslationFeedbackKP,
            drivingConstants.TranslationFeedbackKI,
            drivingConstants.TranslationFeedbackKD,
            drivingConstants.getTranslationConstraints(),
            Hertz.of(50)
        );
    }
    
    @Override
    protected ChassisSpeeds getDrivenVelocity() {
        return rotateToCornerCoordinates(getDrivenVelocityInCornerCoordinates(
            rotateToCornerCoordinates(getSwerveDrive().getPosition2d()),
            rotateToCornerCoordinates(super.getSwerveDrive().getVelocity()),
            rotateToCornerCoordinates(super.getDrivenVelocity())
        ));
    }

    private ChassisSpeeds getDrivenVelocityInCornerCoordinates(Pose2d robotPose, ChassisSpeeds robotVelocity, ChassisSpeeds input) {
        DogLog.log("CornerPose", robotPose);
        DogLog.log("CornerVelocity", robotVelocity);
        DogLog.log("CornerInput", input);

        if (needsBarrierConstraint(robotPose, robotVelocity, input)) {
            double y = robotPose.getY();
            double targetY = getNearestObstacleAvoidingY(y);

            if (Math.abs(y - targetY) < 0.025) {
                return input;
            }

            double vyCorrection = controller.calculate(
                new TrapezoidProfile.State(
                    robotPose.getY(),
                    robotVelocity.vyMetersPerSecond
                ),
                new TrapezoidProfile.State(
                    targetY,
                    0.0
                )
            );

            return new ChassisSpeeds(
                input.vxMetersPerSecond,
                vyCorrection,
                input.omegaRadiansPerSecond
            );
        } else {
            return input;
        }
    }
    
    private boolean needsBarrierConstraint(Pose2d robotPose, ChassisSpeeds robotVelocity, ChassisSpeeds input) {
        double x = robotPose.getX();
        double vxIn = input.vxMetersPerSecond;
        double vx = robotVelocity.vxMetersPerSecond;
        double vy = robotVelocity.vyMetersPerSecond;
        double y = robotPose.getY();

        double a = getSwerveDrive().getConstants().Driving.MaxLinearAcceleration.in(MetersPerSecondPerSecond);

        if (x >= BARRIER_START && x <= BARRIER_END) {
            return true;
        }

        double avoidObstacleY = getNearestObstacleAvoidingY(y);
        double timeToAvoidObstacle = getTimeToTarget(avoidObstacleY - y, vy, a);

        double scaryVx = (x > BARRIER_MIDDLE ? Math.min(0, Math.min(vx, vxIn)) : Math.max(0, Math.max(vx, vxIn)));

        if (scaryVx != 0) {
            double distanceToBarrierMiddle = (x > BARRIER_MIDDLE ? BARRIER_END : BARRIER_START) - x;

            double timeToBarrierMiddle = distanceToBarrierMiddle / scaryVx;

            DogLog.log("TimeToAvoidObstacle", timeToAvoidObstacle);
            DogLog.log("TimeToBarrierMiddle", timeToBarrierMiddle);

            if (timeToAvoidObstacle > timeToBarrierMiddle) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    private double getNearestObstacleAvoidingY(double y) {
        if (y < TRENCH_BUMP_BOUNDARY) {
            return TRENCH_CENTER;
        } else if (y <= BUMP_START) {
            return BUMP_START;
        } else if (y >= BUMP_END) {
            return BUMP_END;
        } else {
            return y;
        }
    }

    private double getTimeToTarget(double initialPosition, double initialVelocity, double acceleration) {
        double a = acceleration / 2.0 * (initialPosition > 0 ? -1 : 1);
        double b = initialVelocity;
        double c = initialPosition;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            throw new IllegalArgumentException("No valid solution for time to target");
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);

        double t1 = (-b + sqrtDiscriminant) / (2 * a);
        double t2 = (-b - sqrtDiscriminant) / (2 * a);

        if (t1 < 0 && t2 < 0) {
            throw new IllegalArgumentException("No valid solution for time to target");
        }

        if (t1 < 0) {
            return t2;
        }

        if (t2 < 0) {
            return t1;
        }

        return Math.min(t1, t2);
    }

    private Pose2d rotateToCornerCoordinates(Pose2d pose) {
        Pose2d robotPose = getSwerveDrive().getPosition2d();

        if (robotPose.getX() > FIELD_LENGTH / 2) {
            robotPose = new Pose2d(
                FIELD_LENGTH - robotPose.getX(),
                robotPose.getY(),
                robotPose.getRotation().times(-1).plus(Rotation2d.fromDegrees(180))
            );
        }

        if (robotPose.getY() > FIELD_WIDTH / 2) {
            robotPose = new Pose2d(
                robotPose.getX(),
                FIELD_WIDTH - robotPose.getY(),
                robotPose.getRotation().times(-1)
            );
        }

        return robotPose;
    }

    private ChassisSpeeds rotateToCornerCoordinates(ChassisSpeeds speeds) {
        Pose2d robotPose = getSwerveDrive().getPosition2d();
        speeds = new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );

        if (robotPose.getX() > FIELD_LENGTH / 2) {
            speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond;
            speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond;
        }

        if (robotPose.getY() > FIELD_WIDTH / 2) {
            speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
            speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond;
        }

        return speeds;
    }
}
