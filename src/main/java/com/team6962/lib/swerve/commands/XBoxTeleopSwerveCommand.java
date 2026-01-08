package com.team6962.lib.swerve.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants.Joystick;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants.Trigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;

/**
 * A teleop swerve command that uses an Xbox controller for driver input. This
 * command provides field-oriented control with configurable speed scaling,
 * boost functionality, and fine control mode using the D-pad.
 * <p>
 * <b>Default Control Scheme:</b>
 * <ul>
 *   <li>Left stick: Translation (field-oriented)</li>
 *   <li>Right stick horizontal axis: Rotation</li>
 *   <li>Right trigger: Boost (increases speed)</li>
 *   <li>Left trigger: Angular super boost (maximizes rotation speed), designed
 *       to dislodge stuck game pieces</li>
 *   <li>D-pad: Fine control mode (robot-relative slow movement)</li>
 * </ul>
 * <p>
 * All control parameters are configured through {@link XBoxTeleopSwerveConstants},
 * including joystick deadband, sensitivity curve, speed scaling factors, and
 * button/trigger assignments.
 */
public class XBoxTeleopSwerveCommand extends TeleopSwerveCommand {
    private XboxController controller;
    private XBoxTeleopSwerveConstants constants;

    /**
     * Constructs an XBoxTeleopSwerveCommand with the specified swerve drive
     * and configuration constants.
     *
     * @param swerveDrive The swerve drive subsystem to control
     * @param constants   The configuration constants for controller mapping
     *                    and speed settings
     */
    public XBoxTeleopSwerveCommand(
        CommandSwerveDrive swerveDrive,
        XBoxTeleopSwerveConstants constants
    ) {
        super(swerveDrive);

        this.controller = new XboxController(constants.ControllerPort);
        this.constants = constants;
    }

    /**
     * Converts fractional power values (0.0 to 1.0) to actual velocities based
     * on the configured maximum velocities.
     *
     * @param fractionMaxSpeeds The chassis speeds as fractions of maximum speed
     * @return The chassis speeds in actual velocity units
     */
    private ChassisSpeeds outputPowerToVelocity(ChassisSpeeds fractionMaxSpeeds) {
        double maxLinearVelocity = getSwerveDrive().getConstants().Driving.MaxLinearVelocity.in(MetersPerSecond);
        double maxAngularVelocity = getSwerveDrive().getConstants().Driving.MaxAngularVelocity.in(RadiansPerSecond);

        return new ChassisSpeeds(
            fractionMaxSpeeds.vxMetersPerSecond * maxLinearVelocity,
            fractionMaxSpeeds.vyMetersPerSecond * maxLinearVelocity,
            fractionMaxSpeeds.omegaRadiansPerSecond * maxAngularVelocity
        );
    }

    /**
     * Reorients the control inputs when running in simulation mode, if this
     * feature is enabled. This rotates the controls so that "up" on the
     * joystick or keyboard moves the robot toward the top of the screen rather
     * than away from the driver station.
     *
     * @param speeds The original chassis speeds
     * @return The reoriented chassis speeds (unchanged if not in simulation or
     *         reorientation is disabled)
     */
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

    /**
     * Computes the desired field-relative velocity based on the current
     * controller inputs. This method handles both normal driving mode and
     * fine control mode (when D-pad is pressed).
     *
     * @return The desired field-relative chassis speeds
     */
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

    /**
     * Gets the translation speed scalar for normal (non-fine-control) mode,
     * interpolated between default and boost speeds based on boost input.
     *
     * @return The translation speed scalar (0.0 to 1.0)
     */
    private double getNonFineControlTranslationScalar() {
        return MathUtil.interpolate(constants.DefaultTranslationalSpeed, constants.BoostTranslationalSpeed, getBoost());
    }

    /**
     * Gets the angular speed scalar for normal (non-fine-control) mode,
     * interpolated based on boost and super boost inputs.
     *
     * @return The angular speed scalar (0.0 to 1.0, or higher with super boost)
     */
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

    /**
     * Gets the translation speed scalar for fine control mode, interpolated
     * between fine control and boosted fine control speeds.
     *
     * @return The fine control translation speed scalar (0.0 to 1.0)
     */
    private double getFineControlTranslationScalar() {
        return MathUtil.interpolate(constants.FineControlTranslationalSpeed, constants.BoostedFineControlTranslationalSpeed, getBoost());
    }

    /**
     * Gets the angular speed scalar for fine control mode, interpolated based
     * on boost and super boost inputs.
     *
     * @return The fine control angular speed scalar (0.0 to 1.0, or higher with
     *         super boost)
     */
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

    /**
     * Gets the current boost value from the configured boost trigger axis.
     *
     * @return The boost value (0.0 when not pressed, up to 1.0 when fully
     *         pressed)
     */
    private double getBoost() {
        return constants.BoostAxis == Trigger.LeftTrigger ? controller.getLeftTriggerAxis() : // Axis 2
            controller.getRightTriggerAxis(); // Axis 3
    }

    /**
     * Gets the current angular super boost value from the configured trigger
     * axis.
     *
     * @return The super boost value (0.0 when not pressed, up to 1.0 when fully
     *         pressed)
     */
    private double getAngularSuperBoost() {
        return constants.AngularSuperBoostAxis == Trigger.LeftTrigger ? controller.getLeftTriggerAxis() : // Axis 3
            controller.getRightTriggerAxis(); // Axis 2
    }

    /**
     * Gets the mapped translation input from the configured translation
     * joystick, with deadband and sensitivity curve applied.
     *
     * @return The mapped translation input as a 2D vector (magnitude 0.0 to 1.0)
     */
    private Translation2d getMappedTranslationInput() {
        Translation2d rawInput = new Translation2d(
            -(constants.TranslationJoystick == Joystick.LeftStick ? controller.getLeftY() : controller.getRightY()), // Axis 1 or 5
            -(constants.TranslationJoystick == Joystick.LeftStick ? controller.getLeftX() : controller.getRightX()) // Axis 0 or 4
        );

        return map2DJoystickInput(rawInput);
    }

    /**
     * Gets the mapped rotation input from the horizontal axis of the configured
     * rotation joystick, with deadband and sensitivity curve applied.
     *
     * @return The mapped rotation input (-1.0 to 1.0)
     */
    private double getMappedRotationInput() {
        double rawInput = constants.RotationJoystick == Joystick.LeftStick ? controller.getLeftX() : controller.getRightX(); // Axis 0 or 4

        return map1DJoystickInput(Math.abs(rawInput)) * Math.signum(rawInput);
    }

    /**
     * Maps a 2D joystick input through the deadband and sensitivity curve. The
     * mapping is applied to the magnitude of the input vector while preserving
     * its direction.
     *
     * @param joystickInput The raw joystick input as a 2D vector
     * @return The mapped joystick input with deadband and sensitivity applied
     */
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

    /**
     * Maps a 1D joystick input through the deadband and sensitivity curve.
     * Inputs below the deadband return 0, and inputs above are scaled from
     * the deadband threshold to 1.0, then raised to the configured exponent.
     *
     * @param input The raw joystick input magnitude (0.0 to 1.0)
     * @return The mapped input value (0.0 to 1.0)
     */
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

    /**
     * Checks if fine control mode is active. Fine control is activated when
     * any D-pad direction is pressed.
     *
     * @return True if the D-pad is pressed in any direction
     */
    private boolean isFineControlling() {
        return controller.getPOV() != -1;
    }

    /**
     * Gets the fine control input direction from the D-pad. Returns a unit
     * vector in the direction of the pressed D-pad button.
     *
     * @return A unit vector in the D-pad direction, or zero if not fine
     *         controlling
     */
    private Translation2d getFineControlInput() {
        if (!isFineControlling()) return new Translation2d();

        Rotation2d direction = Rotation2d.fromDegrees(controller.getPOV()).unaryMinus();
        Translation2d velocity = new Translation2d(1.0, 0.0).rotateBy(direction);

        return velocity;
    }
}
