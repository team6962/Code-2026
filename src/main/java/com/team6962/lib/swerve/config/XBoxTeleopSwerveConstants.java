package com.team6962.lib.swerve.config;

import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * Configuration constants for teleop control of a swerve drive using an Xbox controller. This
 * includes joystick mappings, button assignments, speed settings, and control tuning parameters.
 */
public class XBoxTeleopSwerveConstants implements Cloneable {
  /**
   * The port number of the Xbox controller used to control the robot.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 5
   * </ul>
   */
  public int ControllerPort = 0;

  /** The joystick used to control the robot's translation. */
  public Joystick TranslationJoystick = Joystick.LeftStick;

  /** The joystick used to control the robot's rotation. */
  public Joystick RotationJoystick = Joystick.RightStick;

  /**
   * When pressed, this button will zero the robot's yaw heading, making the robot think it is
   * facing directly away from the driver station. This constant can be set to null to make no
   * button zero the robot's yaw.
   */
  public Button ZeroYawButton = Button.kY;

  /** When held, this button will enable the boost speeds for translation and rotation. */
  public Trigger BoostAxis = Trigger.RightTrigger;

  /**
   * When held, this button will maximize the rotational speed, ignoring the angular speed settings
   * (except the sign of the angular speed, which it will copy). This is useful for quickly turning
   * the robot to dislodge objects.
   */
  public Trigger AngularSuperBoostAxis = Trigger.LeftTrigger;

  /**
   * The translational speed used during teleop when not boosting or using the D-pad for fine
   * control. This is represented as a fraction of the maximum robot speed, where 1.0 is the maximum
   * speed and 0.0 is stationary.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double DefaultTranslationalSpeed = 0.5;

  /**
   * The angular speed used during teleop when not boostin or using the D-pad for fine control. This
   * is represented as a fraction of the maximum robot angular speed, where 1.0 is the maximum
   * angular speed and 0.0 is stationary.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double DefaultAngularSpeed = 0.5;

  /**
   * The translational speed used during teleop when boosting and not using the D-pad for fine
   * control. This is represented as a fraction of the maximum robot speed, where 1.0 is the maximum
   * speed and 0.0 is stationary.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double BoostTranslationalSpeed = 1.0;

  /**
   * The angular speed used during teleop when boosting and not using the D-pad for fine control.
   * This is represented as a fraction of the maximum robot angular speed, where 1.0 is the maximum
   * angular speed and 0.0 is stationary.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double BoostAngularSpeed = 0.5;

  /**
   * When true, the D-pad on the Xbox controller will enable fine control mode, where the robot
   * moves slowly in the direction of the D-pad presses. Fine control is robot centric.
   */
  public boolean EnableFineControl = true;

  /**
   * The translational speed used during teleop when not boosing but using the D-pad for fine
   * control. This is represented as a fraction of the maximum robot speed, where 1.0 is the maximum
   * speed and 0.0 is stationary.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double FineControlTranslationalSpeed = 0.1;

  /**
   * The angular speed used during teleop when not boosting but using the D-pad for fine control.
   * This is represented as a fraction of the maximum robot angular speed, where 1.0 is the maximum
   * angular speed and 0.0 is stationary.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double FineControlAngularSpeed = 0.3;

  /**
   * The translational speed used during teleop when boosting and using the D-pad for fine control.
   * This is represented as a fraction of the maximum robot speed, where 1.0 is the maximum speed
   * and 0.0 is stationary.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double BoostedFineControlTranslationalSpeed = 0.5;

  /**
   * The angular speed used during teleop when boosting and using the D-pad for fine control. This
   * is represented as a fraction of the maximum robot angular speed, where 1.0 is the maximum
   * angular speed and 0.0 is stationary.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double BoostedFineControlAngularSpeed = 0.3;

  /**
   * The power to which joystick inputs are raised for exponential mapping. Higher values result in
   * more sensitivity at low speeds. Values less than 1.0 are not recommended, as they lead to
   * higher sensitivity at low speeds and reduced sensitivity at high speeds.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   * </ul>
   */
  public double JoystickSensitivityExponent = 1.8;

  /**
   * The deadband for the joysticks controlling translation and rotation. Any joystick input with an
   * absolute value less than the deadband will be treated as zero.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.000002
   *   <li><b>Maximum Value:</b> 1.0
   * </ul>
   */
  public double Deadband = 0.1;

  /**
   * Whether the controls will be reoriented in simulation, making up be towards the top of the
   * screen instead of away from the driver station.
   */
  public boolean ReorientControlsInSimulation = true;

  /**
   * Sets the controller port, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param controllerPort The controller port number (0-5)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withControllerPort(int controllerPort) {
    ControllerPort = controllerPort;
    return this;
  }

  /**
   * Sets the translation joystick, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param translationJoystick The joystick for controlling translation
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withTranslationJoystick(Joystick translationJoystick) {
    TranslationJoystick = translationJoystick;
    return this;
  }

  /**
   * Sets the rotation joystick, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param rotationJoystick The joystick for controlling rotation
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withRotationJoystick(Joystick rotationJoystick) {
    RotationJoystick = rotationJoystick;
    return this;
  }

  /**
   * Sets the zero yaw button, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param zeroYawButton The button for zeroing the robot's yaw
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withZeroYawButton(Button zeroYawButton) {
    ZeroYawButton = zeroYawButton;
    return this;
  }

  /**
   * Sets the boost axis, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param boostAxis The trigger for enabling boost speeds
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withBoostAxis(Trigger boostAxis) {
    BoostAxis = boostAxis;
    return this;
  }

  /**
   * Sets the angular super boost axis, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param angularSuperBoostAxis The trigger for enabling maximum rotational speed
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withAngularSuperBoostAxis(Trigger angularSuperBoostAxis) {
    AngularSuperBoostAxis = angularSuperBoostAxis;
    return this;
  }

  /**
   * Sets the default translational speed, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param defaultTranslationalSpeed The default translational speed (0.0-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withDefaultTranslationalSpeed(double defaultTranslationalSpeed) {
    DefaultTranslationalSpeed = defaultTranslationalSpeed;
    return this;
  }

  /**
   * Sets the default angular speed, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param defaultAngularSpeed The default angular speed (0.0-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withDefaultAngularSpeed(double defaultAngularSpeed) {
    DefaultAngularSpeed = defaultAngularSpeed;
    return this;
  }

  /**
   * Sets the boost translational speed, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param boostTranslationalSpeed The boost translational speed (0.0-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withBoostTranslationalSpeed(double boostTranslationalSpeed) {
    BoostTranslationalSpeed = boostTranslationalSpeed;
    return this;
  }

  /**
   * Sets the boost angular speed, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param boostAngularSpeed The boost angular speed (0.0-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withBoostAngularSpeed(double boostAngularSpeed) {
    BoostAngularSpeed = boostAngularSpeed;
    return this;
  }

  /**
   * Sets whether fine control is enabled, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param enableFineControl True if D-pad fine control is enabled
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withEnableFineControl(boolean enableFineControl) {
    EnableFineControl = enableFineControl;
    return this;
  }

  /**
   * Sets the fine control translational speed, and returns this XBoxTeleopSwerveConstants for
   * chaining.
   *
   * @param fineControlTranslationalSpeed The fine control translational speed (0.0-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withFineControlTranslationalSpeed(
      double fineControlTranslationalSpeed) {
    FineControlTranslationalSpeed = fineControlTranslationalSpeed;
    return this;
  }

  /**
   * Sets the fine control angular speed, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param fineControlAngularSpeed The fine control angular speed (0.0-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withFineControlAngularSpeed(double fineControlAngularSpeed) {
    FineControlAngularSpeed = fineControlAngularSpeed;
    return this;
  }

  /**
   * Sets the boosted fine control translational speed, and returns this XBoxTeleopSwerveConstants
   * for chaining.
   *
   * @param boostedFineControlTranslationalSpeed The boosted fine control translational speed
   *     (0.0-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withBoostedFineControlTranslationalSpeed(
      double boostedFineControlTranslationalSpeed) {
    BoostedFineControlTranslationalSpeed = boostedFineControlTranslationalSpeed;
    return this;
  }

  /**
   * Sets the boosted fine control angular speed, and returns this XBoxTeleopSwerveConstants for
   * chaining.
   *
   * @param boostedFineControlAngularSpeed The boosted fine control angular speed (0.0-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withBoostedFineControlAngularSpeed(
      double boostedFineControlAngularSpeed) {
    BoostedFineControlAngularSpeed = boostedFineControlAngularSpeed;
    return this;
  }

  /**
   * Sets the joystick sensitivity exponent, and returns this XBoxTeleopSwerveConstants for
   * chaining.
   *
   * @param joystickSensitivityExponent The exponent for joystick sensitivity (1.0+ recommended)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withJoystickSensitivityExponent(
      double joystickSensitivityExponent) {
    JoystickSensitivityExponent = joystickSensitivityExponent;
    return this;
  }

  /**
   * Sets the deadband, and returns this XBoxTeleopSwerveConstants for chaining.
   *
   * @param deadband The joystick deadband (0.000002-1.0)
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withDeadband(double deadband) {
    Deadband = deadband;
    return this;
  }

  /**
   * Sets whether controls are reoriented in simulation, and returns this XBoxTeleopSwerveConstants
   * for chaining.
   *
   * @param reorientControlsInSimulation True if controls should be reoriented in simulation
   * @return This XBoxTeleopSwerveConstants object
   */
  public XBoxTeleopSwerveConstants withReorientControlsInSimulation(
      boolean reorientControlsInSimulation) {
    ReorientControlsInSimulation = reorientControlsInSimulation;
    return this;
  }

  /** A joystick on the Xbox controller. */
  public static enum Joystick {
    /** The left joystick on the Xbox controller. */
    LeftStick,

    /** The right joystick on the Xbox controller. */
    RightStick
  }

  /** A trigger on the Xbox controller. */
  public static enum Trigger {
    /** The left trigger on the Xbox controller. */
    LeftTrigger,

    /** The right trigger on the Xbox controller. */
    RightTrigger
  }

  @Override
  public XBoxTeleopSwerveConstants clone() {
    try {
      return (XBoxTeleopSwerveConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
