package frc.robot.subsystems.hopper.beltFloor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;

public class BeltFloor extends SubsystemBase {
  private TalonFX BeltFloorMotor;

  public BeltFloor() {
    BeltFloorMotor =
        new TalonFX(
            HopperConstants.BELT_FLOOR_MOTOR_CAN_ID, new CANBus(HopperConstants.CANBUS_NAME));
    BeltFloorMotor.getConfigurator().apply(HopperConstants.BELT_FLOOR_MOTOR_CONFIGURATION);
  }
}
