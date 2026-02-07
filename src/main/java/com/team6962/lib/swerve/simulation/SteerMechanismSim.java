package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

/**
 * Simulates a swerve module's steer mechanism using MapleSim physics simulation.
 *
 * <p>This class bridges between CTRE's TalonFX and CANcoder simulation states and MapleSim's
 * physics simulation. The {@link #updateBeforeArena(double)} method passes the motor controller's
 * applied voltage to the physics simulation and the {@link #updateAfterArena(double)} method
 * updates both the motor controller's and encoder's simulated sensor values based on the physics
 * simulation's results.
 *
 * <p>The simulation handles device inversion by converting between each device's configured
 * positive direction and the physics simulation's counter-clockwise-positive convention. The motor
 * and encoder may have independent inversion settings.
 *
 * <p>The {@link #updateBeforeArena(double)} and {@link #updateAfterArena(double)} methods should be
 * called periodically (typically every simulation tick) to advance the physics simulation and
 * synchronize state with the motor controller and encoder.
 */
public class SteerMechanismSim {
  /** The corner position of this module on the robot. */
  private Corner corner;

  /** Drivetrain configuration containing motor parameters. */
  private DrivetrainConstants constants;

  /** Simulation state of the TalonFX motor controller. */
  private TalonFXSimState motorControllerSimulation;

  /** Simulation state of the CANcoder encoder. */
  private CANcoderSimState encoderSimulation;

  /** MapleSim swerve module simulation instance for this steer mechanism. */
  private SwerveModuleSimulation moduleSim;

  /** MapleSim motor controller simulation instance for this steer mechanism. */
  private GenericMotorController motorSim;

  /**
   * Creates a new steer mechanism simulation.
   *
   * @param corner the corner position of the module
   * @param constants drivetrain configuration
   * @param motorController the TalonFX motor controller to simulate
   * @param encoder the CANcoder encoder to simulate
   * @param mapleSim the MapleSim instance that simulates the swerve drive's physics
   */
  public SteerMechanismSim(
      Corner corner,
      DrivetrainConstants constants,
      TalonFX motorController,
      CANcoder encoder,
      MapleSim mapleSim) {
    this.corner = corner;
    this.constants = constants;

    this.motorControllerSimulation = motorController.getSimState();
    this.encoderSimulation = encoder.getSimState();

    this.moduleSim = mapleSim.getSwerveSim().getModules()[corner.getIndex()];

    motorSim = moduleSim.useGenericControllerForSteer();
  }

  /**
   * Returns the sign multiplier to convert between the motor controller's configured positive
   * direction and the physics simulation's counter-clockwise-positive convention.
   *
   * @return -1.0 if the motor is configured as clockwise-positive, 1.0 if
   *     counter-clockwise-positive
   */
  private double getMotorInversionSign() {
    return constants.SteerMotor.DeviceConfiguration.MotorOutput.Inverted
            == InvertedValue.Clockwise_Positive
        ? -1.0
        : 1.0;
  }

  /**
   * Returns the sign multiplier to convert between the encoder's configured positive direction and
   * the physics simulation's counter-clockwise-positive convention.
   *
   * @return -1.0 if the encoder is configured as clockwise-positive, 1.0 if
   *     counter-clockwise-positive
   */
  private double getEncoderInversionSign() {
    return constants.SteerEncoder.DeviceConfiguration.MagnetSensor.SensorDirection
            == SensorDirectionValue.Clockwise_Positive
        ? -1.0
        : 1.0;
  }

  /**
   * Returns the configured steer encoder offset in mechanism coordinates.
   *
   * @return the encoder offset angle
   */
  private Angle getEncoderOffset() {
    return constants.getSwerveModule(corner).SteerEncoderOffset.minus(corner.getRotation());
  }

  /**
   * Passes the motor controller's applied voltage to the physics simulation before the arena
   * update.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void updateBeforeArena(double deltaTimeSeconds) {
    // Set the physics simulation input voltage to the motor controller's
    // applied output voltage
    motorSim.requestVoltage(
        motorControllerSimulation.getMotorVoltageMeasure().times(getMotorInversionSign()));
  }

  /**
   * Passes data from the physics simulation back to the motor controller and encoder simulations
   * after the arena update.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void updateAfterArena(double deltaTimeSeconds) {
    Angle offset = getEncoderOffset();

    // Update the motor controller simulation with the new position and
    // velocity from the physics simulation
    motorControllerSimulation.setRawRotorPosition(
        moduleSim
            .getSteerAbsoluteFacing()
            .getMeasure()
            .minus(offset)
            .times(constants.SteerMotor.GearReduction * getMotorInversionSign()));
    motorControllerSimulation.setRotorVelocity(
        moduleSim
            .getSteerAbsoluteEncoderSpeed()
            .times(constants.SteerMotor.GearReduction * getMotorInversionSign()));

    // Update the encoder simulation with the new position and velocity from
    // the physics simulation
    encoderSimulation.setRawPosition(
        moduleSim
            .getSteerAbsoluteFacing()
            .getMeasure()
            .minus(offset)
            .times(getEncoderInversionSign()));
    encoderSimulation.setVelocity(
        moduleSim.getSteerAbsoluteEncoderSpeed().times(getEncoderInversionSign()));

    DogLog.log(
        "Drivetrain/Simulation/" + corner.name() + "Steer/Position",
        moduleSim.getSteerAbsoluteFacing().getMeasure().in(Radians));
  }
}
