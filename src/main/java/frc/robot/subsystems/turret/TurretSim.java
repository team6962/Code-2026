package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** This handles the TurretRotation behavior in simulation mode */
public class TurretSim {
  // This defines the motor and physics simulators
  private TalonFXSimState motorSim;
  private DCMotorSim physicsSim;

  // This assigns motor and physics simulators to a digital motor
  // and a configured physics simulation respectively
  public TurretSim(TalonFX motor) {
    motorSim = motor.getSimState();
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(TurretConstants.NUM_MOTORS),
            TurretConstants.MOMENT_OF_INERTIA,
            TurretConstants.SENSOR_TO_MECHANISM_RATIO),
            DCMotor.getKrakenX60Foc(TurretConstants.NUM_MOTORS));
  }

  /** This updates the simulated motors periodically */
  public void update() {
    // Sets the correct voltage values in the simulations and gets the angular velocity and position
    // angle
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    double motorVoltage = invert(motorSim.getMotorVoltage(), false);
    Angle position = physicsSim.getAngularPosition();
    AngularVelocity velocity = RadiansPerSecond.of(physicsSim.getAngularVelocityRadPerSec());

    // Handles physics simulation updates
    physicsSim.setInput(motorVoltage);
    physicsSim.update(TurretConstants.SIMULATION_UPDATE_PERIOD);
    motorSim.setRawRotorPosition(invert(position, false).times(TurretConstants.SENSOR_TO_MECHANISM_RATIO));
    motorSim.setRotorVelocity(invert(velocity, false).times(TurretConstants.SENSOR_TO_MECHANISM_RATIO));
  }

  /** Defines helper methods */
  private static Angle invert(Angle angle, boolean shouldBeInverted) {
    return shouldBeInverted ? angle.unaryMinus() : angle;
  }

  private static AngularVelocity invert(AngularVelocity velocity, boolean shouldBeInverted) {
    return shouldBeInverted ? velocity.unaryMinus() : velocity;
  }

  public static double invert(double value, boolean shouldBeInverted) {
    return shouldBeInverted ? -value : value;
  }
}