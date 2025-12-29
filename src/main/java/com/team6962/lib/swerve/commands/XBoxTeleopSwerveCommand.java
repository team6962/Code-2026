package com.team6962.lib.swerve.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants.Trigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;

public class XBoxTeleopSwerveCommand extends TeleopSwerveCommand {
    private XboxController controller;
    private XBoxTeleopSwerveConstants constants;
    
    public XBoxTeleopSwerveCommand(
        CommandSwerveDrive swerveDrive,
        XBoxTeleopSwerveConstants constants
    ) {
        super(swerveDrive);

        this.controller = new XboxController(constants.ControllerPort);
        this.constants = constants;
    }

    private ChassisSpeeds outputPowerToVelocity(ChassisSpeeds fractionMaxSpeeds) {
        double maxLinearVelocity = getSwerveDrive().getConstants().Driving.MaxLinearVelocity.in(MetersPerSecond);
        double maxAngularVelocity = getSwerveDrive().getConstants().Driving.MaxAngularVelocity.in(RadiansPerSecond);

        return new ChassisSpeeds(
            fractionMaxSpeeds.vxMetersPerSecond * maxLinearVelocity,
            fractionMaxSpeeds.vyMetersPerSecond * maxLinearVelocity,
            fractionMaxSpeeds.omegaRadiansPerSecond * maxAngularVelocity
        );
    }

    private ChassisSpeeds reorientInSimulation(ChassisSpeeds speeds) {
        if (constants.ReorientControlsInSimulation && RobotBase.isSimulation()) {
            return new ChassisSpeeds(
                -speeds.vyMetersPerSecond,
                speeds.vxMetersPerSecond,
                speeds.omegaRadiansPerSecond
            );
        } else {
            return speeds;
        }
    }

    @Override
    protected ChassisSpeeds getDrivenVelocity() {
        if (isFineControlling()) {
            Translation2d robotRelativeVelocity = getFineControlInput();

            robotRelativeVelocity = robotRelativeVelocity.times(getFineControlTranslationScalar());

            double angularVelocity = getMappedRotationInput() * getFineControlAngularScalar();

            return reorientInSimulation(outputPowerToVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeVelocity.getX(),
                robotRelativeVelocity.getY(),
                angularVelocity,
                new Rotation2d(getSwerveDrive().getHeading())
            )));
        } else {
            Translation2d robotRelativeVelocity = getMappedTranslationInput();

            robotRelativeVelocity = robotRelativeVelocity.times(getNonFineControlTranslationScalar());

            double angularVelocity = getMappedRotationInput() * getNonFineControlAngularScalar();

            return reorientInSimulation(outputPowerToVelocity(new ChassisSpeeds(
                robotRelativeVelocity.getX(),
                robotRelativeVelocity.getY(),
                angularVelocity
            )));
        }
    }

    private double getNonFineControlTranslationScalar() {
        return MathUtil.interpolate(constants.DefaultTranslationalSpeed, constants.BoostTranslationalSpeed, getBoost());
    }

    private double getNonFineControlAngularScalar() {
        return MathUtil.interpolate(
            MathUtil.interpolate(
                constants.DefaultAngularSpeed,
                constants.BoostAngularSpeed,
                getBoost()
            ),
            Math.signum(constants.DefaultAngularSpeed),
            getAngularSuperBoost()
        );
    }

    private double getFineControlTranslationScalar() {
        return MathUtil.interpolate(constants.FineControlTranslationalSpeed, constants.BoostedFineControlTranslationalSpeed, getBoost());
    }

    private double getFineControlAngularScalar() {
        return MathUtil.interpolate(
            MathUtil.interpolate(
                constants.FineControlAngularSpeed,
                constants.BoostedFineControlAngularSpeed,
                getBoost()
            ),
            Math.signum(constants.FineControlAngularSpeed),
            getAngularSuperBoost()
        );
    }

    private double getBoost() {
        return constants.BoostAxis == Trigger.LeftTrigger ? controller.getLeftTriggerAxis() : // Axis 2
            controller.getRightTriggerAxis(); // Axis 3
    }

    private double getAngularSuperBoost() {
        return constants.AngularSuperBoostAxis == Trigger.LeftTrigger ? controller.getLeftTriggerAxis() : // Axis 3
            controller.getRightTriggerAxis(); // Axis 2
    }

    private Translation2d getMappedTranslationInput() {
        Translation2d rawInput = new Translation2d(
            -controller.getLeftY(), // Axis 1
            -controller.getLeftX() // Axis 0
        );

        return map2DJoystickInput(rawInput);
    }

    private double getMappedRotationInput() {
        double rawInput = controller.getRightX(); // Axis 4

        return map1DJoystickInput(Math.abs(rawInput)) * Math.signum(rawInput);
    }

    private Translation2d map2DJoystickInput(Translation2d joystickInput) {
        // Calculate the magnitude of the joystick input using the Pythagorean
        // theorem
        double originalMagnitude = joystickInput.getNorm();

        // If the magnitude is within the deadband, return a zero vector
        if (originalMagnitude < Math.max(2e-6, constants.Deadband)) {
            return new Translation2d();
        }

        // Map the magnitude using the 1D joystick mapping function
        double mappedMagnitude = map1DJoystickInput(originalMagnitude);

        // Return the scaled joystick input vector
        return joystickInput.times(mappedMagnitude / originalMagnitude);
    }

    private double map1DJoystickInput(double input) {
        // Ignore input magnitudes less than the deadband threshold
        if (input < Math.max(2e-6, constants.Deadband)) {
            return 0;
        }

        // Map inputs from [deadband threshold, 1.0] to [0.0, 1.0]
        double output = (input - constants.Deadband) / (1.0 - constants.Deadband);

        // Prevent the output from potentially exceeding 1.0 on diagonal
        // inputs
        output = Math.min(output, 1.0);

        // Scale the output using the configured exponent for finer control
        // at low speeds
        output = Math.pow(output, constants.JoystickSensitivityExponent);

        return output;
    }

    private boolean isFineControlling() {
        return controller.getPOV() != -1;
    }

    private Translation2d getFineControlInput() {
        if (!isFineControlling()) return new Translation2d();

        Rotation2d direction = Rotation2d.fromDegrees(controller.getPOV()).unaryMinus();
        Translation2d velocity = new Translation2d(1.0, 0.0).rotateBy(direction);

        return velocity;
    }
}
