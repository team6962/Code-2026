package com.team6962.lib.swerve.config;

import edu.wpi.first.wpilibj.XboxController.Button;

public class XBoxTeleopSwerveConstants {
    /**
     * The port number of the Xbox controller used to control the robot.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 5
     * </ul>
     */
    public int ControllerPort = 0;

    /**
     * The joystick used to control the robot's translation.
     */
    public Joystick TranslationJoystick = Joystick.LeftStick;

    /**
     * The joystick used to control the robot's rotation.
     */
    public Joystick RotationJoystick = Joystick.RightStick;

    /**
     * When pressed, this button will zero the robot's yaw heading, making the
     * robot think it is facing directly away from the driver station.
     */
    public Button ZeroYawButton = Button.kY;

    /**
     * When held, this button will enable the boost speeds for translation and
     * rotation.
     */
    public Trigger BoostAxis = Trigger.RightTrigger;

    /**
     * When held, this button will maximize the rotational speed, ignoring the
     * angular speed settings (except the sign of the angular speed, which it
     * will copy). This is useful for quickly turning the robot to dislodge
     * objects.
     */
    public Trigger AngularSuperBoostAxis = Trigger.LeftTrigger;

    /**
     * The translational speed used during teleop when not boosting or using the
     * D-pad for fine control. This is represented as a fraction of the maximum
     * robot speed, where 1.0 is the maximum speed and 0.0 is stationary.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double DefaultTranslationalSpeed = 0.5;

    /**
     * The angular speed used during teleop when not boostin or using the D-pad
     * for fine control. This is represented as a fraction of the maximum
     * robot angular speed, where 1.0 is the maximum angular speed and 0.0 is
     * stationary.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double DefaultAngularSpeed = 0.3;

    /**
     * The translational speed used during teleop when boosting and not using
     * the D-pad for fine control. This is represented as a fraction of the
     * maximum robot speed, where 1.0 is the maximum speed and 0.0 is
     * stationary.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double BoostTranslationalSpeed = 1.0;

    /**
     * The angular speed used during teleop when boosting and not using the
     * D-pad for fine control. This is represented as a fraction of the maximum
     * robot angular speed, where 1.0 is the maximum angular speed and 0.0 is
     * stationary.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double BoostAngularSpeed = 0.5;

    /**
     * When true, the D-pad on the Xbox controller will enable fine control
     * mode, where the robot moves slowly in the direction of the D-pad presses.
     * Fine control is robot centric.
     */
    public boolean EnableFineControl = true;

    /**
     * The translational speed used during teleop when not boosing but using
     * the D-pad for fine control. This is represented as a fraction of the
     * maximum robot speed, where 1.0 is the maximum speed and 0.0 is
     * stationary.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double FineControlTranslationalSpeed = 0.1;

    /**
     * The angular speed used during teleop when not boosting but using the
     * D-pad for fine control. This is represented as a fraction of the
     * maximum robot angular speed, where 1.0 is the maximum angular speed and
     * 0.0 is stationary.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double FineControlAngularSpeed = 0.3;

    /**
     * The translational speed used during teleop when boosting and using the
     * D-pad for fine control. This is represented as a fraction of the
     * maximum robot speed, where 1.0 is the maximum speed and 0.0 is
     * stationary.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double BoostedFineControlTranslationalSpeed = 0.5;

    /**
     * The angular speed used during teleop when boosting and using the D-pad
     * for fine control. This is represented as a fraction of the maximum
     * robot angular speed, where 1.0 is the maximum angular speed and 0.0 is
     * stationary.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double BoostedFineControlAngularSpeed = 0.3;

    /**
     * The power to which joystick inputs are raised for exponential mapping.
     * Higher values result in more sensitivity at low speeds. Values less than
     * 1.0 are not recommended, as they lead to higher sensitivity at low speeds
     * and reduced sensitivity at high speeds.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     * </ul>
     */
    public double JoystickSensitivityExponent = 1.8;

    /**
     * The deadband for the joysticks controlling translation and rotation. Any
     * joystick input with an absolute value less than the deadband will be
     * treated as zero.
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.000002
     *   <li> <b>Maximum Value:</b> 1.0
     * </ul>
     */
    public double Deadband = 0.1;

    /**
     * Whether the controls will be reoriented in simulation, making up be
     * towards the top of the screen instead of away from the driver station.
     */
    public boolean ReorientControlsInSimulation = true;

    /**
     * A joystick on the Xbox controller.
     */
    public static enum Joystick {
        /**
         * The left joystick on the Xbox controller.
         */
        LeftStick,

        /**
         * The right joystick on the Xbox controller.
         */
        RightStick
    }

    /**
     * A trigger on the Xbox controller.
     */
    public static enum Trigger {
        /**
         * The left trigger on the Xbox controller.
         */
        LeftTrigger,

        /**
         * The right trigger on the Xbox controller.
         */
        RightTrigger
    }
}
