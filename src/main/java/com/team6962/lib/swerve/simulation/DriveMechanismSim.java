package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.math.WheelMath;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulates a swerve module's drive mechanism using WPILib's DCMotorSim physics simulation.
 *
 * <p>This class bridges the gap between CTRE's TalonFX simulation state and WPILib's physics
 * simulation. It reads the motor controller's applied voltage, runs the physics simulation, and
 * updates the motor controller's simulated sensor values accordingly.
 *
 * <p>The simulation properly handles motor inversion by converting between the motor controller's
 * configured positive direction and the physics simulation's counter-clockwise-positive convention.
 *
 * <p>Getter methods return both angular values (wheel-relative) and linear values (movement of
 * wheel along ground) by converting with the configured wheel radius.
 *
 * <p>The {@link #update(double)} method should be called periodically (typically every simulation
 * tick) to advance the physics simulation and synchronize state with the motor controller.
 */
public class DriveMechanismSim {
  /** The corner of the robot that this module occupies. */
  private Corner corner;

  /** Drivetrain configuration containing motor and wheel constants. */
  private DrivetrainConstants constants;

  /** CTRE simulation state for the TalonFX motor controller. */
  private TalonFXSimState motorControllerSimulation;

  /** WPILib physics simulation for the DC motor. */
  private DCMotorSim physicsSimulation;

  /**
   * Creates a new drive mechanism simulation.
   *
   * @param corner the corner of the robot that this module occupies
   * @param constants drivetrain configuration
   * @param motorController the TalonFX motor controller to simulate
   */
  public DriveMechanismSim(Corner corner, DrivetrainConstants constants, TalonFX motorController) {
    this.corner = corner;
    this.constants = constants;

    this.motorControllerSimulation = motorController.getSimState();

    this.physicsSimulation =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                constants.DriveMotor.SimulatedMotor,
                constants.DriveMotor.SimulatedMomentOfInertia.in(KilogramSquareMeters),
                constants.DriveMotor.GearReduction),
            constants.DriveMotor.SimulatedMotor);
  }

  /**
   * Returns the angular position of the wheel around its shaft.
   *
   * @return The {@link Angle}
   */
  public Angle getAngularPosition() {
    return physicsSimulation.getAngularPosition();
  }

  /**
   * Returns the angular velocity of the wheel around its shaft.
   *
   * @return The {@link AngularVelocity}
   */
  public AngularVelocity getAngularVelocity() {
    return physicsSimulation.getAngularVelocity();
  }

  /** Returns the angular acceleration of the wheel around its shaft. */
  public AngularAcceleration getAngularAcceleration() {
    return physicsSimulation.getAngularAcceleration();
  }

  /** Returns the linear distance traveled by the wheel. */
  public Distance getPosition() {
    return WheelMath.toLinear(
        physicsSimulation.getAngularPosition(), constants.getWheelRadius(corner));
  }

  /** Returns the linear velocity of the wheel along the ground. */
  public LinearVelocity getVelocity() {
    return WheelMath.toLinear(
        physicsSimulation.getAngularVelocity(), constants.getWheelRadius(corner));
  }

  /** Returns the linear acceleration of the wheel along the ground. */
  public LinearAcceleration getAcceleration() {
    return WheelMath.toLinear(
        physicsSimulation.getAngularAcceleration(), constants.getWheelRadius(corner));
  }

  /**
   * Updates the physics simulation and synchronizes motor controller state.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void update(double deltaTimeSeconds) {
    // Set the motor controller's supply voltage to the battery voltage
    // reported by RobotController, which can be overriden in simulation
    motorControllerSimulation.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Sign multiplier to account for motor inversion setting. The motor
    // controller reports values in its configured positive direction, but
    // the physics simulation always uses counter-clockwise-positive, so we
    // need to negate values when the motor is configured as
    // clockwise-positive.
    double motorSign =
        constants.DriveMotor.DeviceConfiguration.MotorOutput.Inverted
                == InvertedValue.Clockwise_Positive
            ? -1.0
            : 1.0;

    // Set the physics simulation input voltage to the motor controller's
    // applied output voltage
    physicsSimulation.setInputVoltage(motorControllerSimulation.getMotorVoltage() * motorSign);

    // Update the physics simulation
    physicsSimulation.update(deltaTimeSeconds);

    // Update the motor controller simulation with the new position,
    // velocity, and acceleration from the physics simulation
    motorControllerSimulation.setRawRotorPosition(
        physicsSimulation
            .getAngularPosition()
            .times(constants.DriveMotor.GearReduction * motorSign));
    motorControllerSimulation.setRotorVelocity(
        physicsSimulation
            .getAngularVelocity()
            .times(constants.DriveMotor.GearReduction * motorSign));
    motorControllerSimulation.setRotorAcceleration(
        physicsSimulation
            .getAngularAcceleration()
            .times(constants.DriveMotor.GearReduction * motorSign));

    DogLog.log(
        "Drivetrain/Simulation/" + corner.name() + "Drive/Position", getPosition().in(Meters));
  }
}
