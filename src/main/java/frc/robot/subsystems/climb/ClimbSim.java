package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.simulation.LinearMechanismSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

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
        Radians.of(
            physicsSim.getPositionMeters()
                / ClimbConstants.DRUM_RADIUS.in(Meters)
                * ClimbConstants.GEAR_RATIO));
    climbMotor.setRotorVelocity(
        RadiansPerSecond.of(
            physicsSim.getVelocityMetersPerSecond()
                / ClimbConstants.DRUM_RADIUS.in(Meters)
                * ClimbConstants.GEAR_RATIO));
    climbMotor.setRotorAcceleration(
        RadiansPerSecondPerSecond.of(
            physicsSim.getVelocityMetersPerSecond()
                / ClimbConstants.DRUM_RADIUS.in(Meters)
                * ClimbConstants.GEAR_RATIO));
  }
}
