package frc.robot.subsystems.hopper.beltFloor;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



import com.ctre.phoenix6.CANBus;

public class BeltFloor extends SubsystemBase{
    private TalonFX BeltFloorMotor;

    public BeltFloor() {
        BeltFloorMotor = 
            new TalonFX(BeltFloorConstants.BELT_FLOOR_MOTOR_CAN_ID, new CANBus(BeltFloorConstants.CANBUS_NAME));
        BeltFloorMotor.getConfigurator().apply(BeltFloorConstants.MOTOR_CONFIGURATION);
    }
}
