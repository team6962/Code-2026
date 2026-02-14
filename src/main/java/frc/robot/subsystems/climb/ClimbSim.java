package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.simulation.LinearMechanismSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

/** climb sim runs a physics sim for the climb mechanism */
public class ClimbSim {
  private TalonFXSimState climbMotor;
  private LinearMechanismSim physicsSim;

  public ClimbSim(TalonFX motor) {
    climbMotor = motor.getSimState();
    physicsSim =
        new LinearMechanismSim(
            DCMotor.getKrakenX60Foc(1),
            ClimbConstants.GEAR_RATIO,
            ClimbConstants.MASS,
            ClimbConstants.DRUM_RADIUS,
            ClimbConstants.MIN_HEIGHT,
            ClimbConstants.MAX_HEIGHT,
            ClimbConstants.CONSTANT_ACCELERATION,
            ClimbConstants.MIN_HEIGHT);
  }

  public void update() {
    climbMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(climbMotor.getMotorVoltage());
    climbMotor.setRawRotorPosition(
        Rotations.of(
            physicsSim.getPositionMeters()
                / ClimbConstants.DRUM_RADIUS.in(Meters)
                * ClimbConstants.GEAR_RATIO));
    climbMotor.setRotorVelocity(
        RotationsPerSecond.of(
            physicsSim.getVelocityMetersPerSecond()
                / ClimbConstants.DRUM_RADIUS.in(Meters)
                * ClimbConstants.GEAR_RATIO));
    climbMotor.setRotorAcceleration(
        RotationsPerSecondPerSecond.of(
            physicsSim.getVelocityMetersPerSecond()
                / ClimbConstants.DRUM_RADIUS.in(Meters)
                * ClimbConstants.GEAR_RATIO));
  }
}
