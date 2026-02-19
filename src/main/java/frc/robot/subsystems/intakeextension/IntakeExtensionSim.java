package frc.robot.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.simulation.LinearMechanismSim;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeExtensionSim {

  private TalonFXSimState motorSim;
  private LinearMechanismSim physicsSim;

  public IntakeExtensionSim(TalonFX motor) {
    motorSim = motor.getSimState();
    physicsSim =
        new LinearMechanismSim(
            IntakeExtensionConstants.MOTOR_PHYSICS,
            IntakeExtensionConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio,
            IntakeExtensionConstants.MOVING_MASS,
            IntakeExtensionConstants.PINION_RADIUS,
            IntakeExtensionConstants.MIN_POSITION,
            IntakeExtensionConstants.MAX_POSITION,
            IntakeExtensionConstants.ANGLE,
            IntakeExtensionConstants.MIN_POSITION);
  }

  public void update() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    double motorVoltage =
        invert(
            motorSim.getMotorVoltage(),
            IntakeExtensionConstants.MOTOR_CONFIGURATION.MotorOutput.Inverted
                == InvertedValue.Clockwise_Positive);
    physicsSim.setInput(motorVoltage);
    physicsSim.update(0.02);
    Angle position =
        Radians.of(
            physicsSim.getPositionMeters() / IntakeExtensionConstants.PINION_RADIUS.in(Meters));
    AngularVelocity velocity =
        RadiansPerSecond.of(
            physicsSim.getVelocityMetersPerSecond()
                / IntakeExtensionConstants.PINION_RADIUS.in(Meters));
    motorSim.setRawRotorPosition(
        invert(
                position,
                IntakeExtensionConstants.MOTOR_CONFIGURATION.MotorOutput.Inverted
                    == InvertedValue.Clockwise_Positive)
            .times(IntakeExtensionConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio));
    motorSim.setRotorVelocity(
        invert(
                velocity,
                IntakeExtensionConstants.MOTOR_CONFIGURATION.MotorOutput.Inverted
                    == InvertedValue.Clockwise_Positive)
            .times(IntakeExtensionConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio));
  }

  private static double invert(double value, boolean shouldBeInverted) {
    return shouldBeInverted ? -value : value;
  }

  private static Angle invert(Angle angle, boolean shouldBeInverted) {
    return shouldBeInverted ? angle.unaryMinus() : angle;
  }

  private static AngularVelocity invert(AngularVelocity velocity, boolean shouldBeInverted) {
    return shouldBeInverted ? velocity.unaryMinus() : velocity;
  }
}
