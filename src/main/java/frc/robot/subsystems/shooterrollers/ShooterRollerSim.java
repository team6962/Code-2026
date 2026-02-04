package frc.robot.subsystems.shooterrollers;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterRollerSim {
  private TalonFXSimState motorSim;
  private DCMotorSim physicsSim;

  public ShooterRollerSim(TalonFX motor) {
    motorSim = motor.getSimState();
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterRollerConstants.MOTOR_PHYSICS,
                // note: ask build what motor will be used
                // update: I guessed right first time so don't change it
                ShooterRollerConstants.MOMENT_OF_INERTIA.in(KilogramSquareMeters),
                ShooterRollerConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio),
            ShooterRollerConstants.MOTOR_PHYSICS);
  }

  public void update() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    double motorVoltage = invert(motorSim.getMotorVoltage(), false);
    Angle position = physicsSim.getAngularPosition();
    AngularVelocity velocity = RadiansPerSecond.of(physicsSim.getAngularVelocityRadPerSec());
    physicsSim.setInput(motorVoltage);
    physicsSim.update(0.02);
    motorSim.setRawRotorPosition(
        invert(position, false)
            .times(ShooterRollerConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio));
    motorSim.setRotorVelocity(
        invert(velocity, false)
            .times(ShooterRollerConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio));
  }

  private static Angle invert(Angle angle, boolean shouldBeInverted) {
    return shouldBeInverted ? angle.unaryMinus() : angle;
  }

  private static AngularVelocity invert(AngularVelocity velocity, boolean shouldBeInverted) {
    return shouldBeInverted ? velocity.unaryMinus() : velocity;
  }

  private static double invert(double value, boolean shouldBeInverted) {
    return shouldBeInverted ? -value : value;
  }
}
