package frc.robot.subsystems.hopper.beltfloor;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.hopper.HopperConstants;

/** class for simulator for the belt floor */
public class BeltFloorSim {
  private TalonFXSimState motorSim;
  private DCMotorSim physicsSim;

  /** simulator for the belt floor */
  public BeltFloorSim(TalonFX motor) {
    motorSim = motor.getSimState();
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HopperConstants.BELT_FLOOR_MOTOR_PHYSICS,
                HopperConstants.BELT_FLOOR_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
                HopperConstants.BELT_FLOOR_MOTOR_CONFIG.Feedback.SensorToMechanismRatio),
            HopperConstants.BELT_FLOOR_MOTOR_PHYSICS);
  }

  /** updates the simulation */
  public void update() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    double motorVoltage = invert(motorSim.getMotorVoltage(), false);
    Angle position = physicsSim.getAngularPosition();
    AngularVelocity velocity = RadiansPerSecond.of(physicsSim.getAngularVelocityRadPerSec());
    physicsSim.setInput(motorVoltage);
    physicsSim.update(0.02);
    motorSim.setRawRotorPosition(
        invert(position, false)
            .times(HopperConstants.BELT_FLOOR_MOTOR_CONFIG.Feedback.SensorToMechanismRatio));
    motorSim.setRotorVelocity(
        invert(velocity, false)
            .times(HopperConstants.BELT_FLOOR_MOTOR_CONFIG.Feedback.SensorToMechanismRatio));
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
