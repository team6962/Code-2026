package com.team6962.lib.swerve.config;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.team6962.lib.math.MeasureUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/** The constants that define the physical structure of the drivetrain. */
public class StructureConstants implements Cloneable {
  /** The wheelbase of the robot, which is the distance between the front and back wheels. */
  public Distance WheelBase = Inches.of(22.75);

  /** The track width of the robot, which is the distance between left and right wheels. */
  public Distance TrackWidth = Inches.of(22.75);

  /**
   * The radius of the wheels used on the robot. This can be overriden for individual swerve modules
   * in their {@link SwerveModuleConstants}.
   */
  public Distance WheelRadius = Inches.of(1.95);

  /**
   * The coefficient of friction between the wheels and the ground. This is used for simulation and
   * can be tuned to match the actual performance of the robot on the field. A typical value for
   * rubber wheels on carpet is around 1.0 to 1.5, but this can vary based on the specific wheel
   * material and field surface.
   */
  public double WheelCOF = 1.426;

  /** The outer length of the robot from bumper to bumper, in the X direction (front to back). */
  public Distance OuterLength = Inches.of(35);

  /** The outer width of the robot from bumper to bumper, in the Y direction (side to side). */
  public Distance OuterWidth = Inches.of(35);

  /** The mass of the robot. */
  public Mass RobotMass = Pounds.of(130);

  /** The moment of inertia of the robot. */
  public MomentOfInertia RobotMomentOfInertia =
      MeasureUtil.toMomentOfInertia(OuterWidth.times(OuterLength).times(RobotMass).div(12));

  public StructureConstants() {}

  /**
   * Sets the wheelbase of the robot (the distance between the front and back wheels), and returns
   * this StructureConstants object for chaining.
   *
   * @param wheelBase The wheelbase of the robot
   * @return This StructureConstants object
   */
  public StructureConstants withWheelBase(Distance wheelBase) {
    WheelBase = wheelBase;
    return this;
  }

  /**
   * Sets the track width of the robot (the distance between left and right wheels), and returns
   * this StructureConstants object for chaining.
   *
   * @param trackWidth The track width of the robot
   * @return This StructureConstants object
   */
  public StructureConstants withTrackWidth(Distance trackWidth) {
    TrackWidth = trackWidth;
    return this;
  }

  /**
   * Sets the radius of the wheels, and returns this StructureConstants object for chaining. This
   * radius can be overriden for specific modules in their {@link SwerveModuleConstants}.
   *
   * @param wheelRadius The wheel radius
   * @return This StructureConstants object
   */
  public StructureConstants withWheelRadius(Distance wheelRadius) {
    WheelRadius = wheelRadius;
    return this;
  }

  /**
   * Sets the coefficient of friction between the wheels and the ground, and returns this
   * StructureConstants object for chaining. This is used for simulation and can be tuned to match
   * the actual performance of the robot on the field.
   *
   * @param wheelCOF The coefficient of friction for the wheels
   * @return This StructureConstants object
   */
  public StructureConstants withWheelCOF(double wheelCOF) {
    WheelCOF = wheelCOF;
    return this;
  }

  /**
   * Sets the outer length of the robot from bumper to bumper in the X direction (front to back),
   * and returns this StructureConstants object for chaining.
   *
   * @param outerLength The outer length of the robot
   * @return This StructureConstants object
   */
  public StructureConstants withOuterLength(Distance outerLength) {
    OuterLength = outerLength;
    return this;
  }

  /**
   * Sets the outer width of the robot from bumper to bumper in the Y direction (side to side), and
   * returns this StructureConstants object for chaining.
   *
   * @param outerWidth The outer width of the robot
   * @return This StructureConstants object
   */
  public StructureConstants withOuterWidth(Distance outerWidth) {
    OuterWidth = outerWidth;
    return this;
  }

  /**
   * Sets the mass of the robot, and returns this StructureConstants object for chaining.
   *
   * @param robotMass The mass of the robot
   * @return This StructureConstants object
   */
  public StructureConstants withRobotMass(Mass robotMass) {
    RobotMass = robotMass;
    return this;
  }

  /**
   * Sets the moment of inertia of the robot, and returns this StructureConstants object for
   * chaining.
   *
   * @param robotMomentOfInertia The moment of inertia of the robot
   * @return This StructureConstants object
   */
  public StructureConstants withRobotMomentOfInertia(MomentOfInertia robotMomentOfInertia) {
    RobotMomentOfInertia = robotMomentOfInertia;
    return this;
  }

  /**
   * Estimates the moment of inertia of the robot using the formula for a rectangular prism: (width
   * * length * mass) / 12. This method updates the RobotMOI field and returns this
   * StructureConstants object for chaining.
   *
   * @return This StructureConstants object
   */
  public StructureConstants estimateMomentOfInertia() {
    RobotMomentOfInertia =
        MeasureUtil.toMomentOfInertia(OuterWidth.times(OuterLength).times(RobotMass).div(12));
    return this;
  }

  /**
   * Gets the swerve drive kinematics for the drivetrain based on the configured wheelbase and track
   * width.
   *
   * @return The swerve drive kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(
        new Translation2d(WheelBase.div(2), TrackWidth.div(2)),
        new Translation2d(WheelBase.div(2), TrackWidth.div(-2)),
        new Translation2d(WheelBase.div(-2), TrackWidth.div(2)),
        new Translation2d(WheelBase.div(-2), TrackWidth.div(-2)));
  }

  @Override
  public StructureConstants clone() {
    try {
      return (StructureConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
