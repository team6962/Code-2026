package frc.robot.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class IntakeExtensionSim {
  private TalonFXSimState motorSim;
  private AngularVelocity lastVelocity = RadiansPerSecond.of(0);
  private ElevatorSim physicsSim;

  public IntakeExtensionSim(TalonFX motor) {
    motorSim = motor.getSimState();
    physicsSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                (IntakeExtensionConstants.MOTOR_PHYSICS),
                IntakeExtensionConstants.MOVING_MASS.in(Kilograms),
                0.05,
                IntakeExtensionConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio),
            IntakeExtensionConstants.MOTOR_PHYSICS,
            IntakeExtensionConstants.MIN_POSITION.in(Meters),
            IntakeExtensionConstants.MAX_POSITION.in(Meters),
            true,
            0);
  }

  public void update() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    double motorVoltage = invert(motorSim.getMotorVoltage(), false);
    Angle position = Radians.of(physicsSim.getPositionMeters());
    AngularVelocity velocity = RadiansPerSecond.of(physicsSim.getVelocityMetersPerSecond());
    physicsSim.setInput(motorVoltage);
    physicsSim.update(0.02);
    motorSim.setRawRotorPosition(invert(position, false).times(5.0 / 0.05));
    motorSim.setRotorVelocity(invert(velocity, false).times(5.0 / 0.05));
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
