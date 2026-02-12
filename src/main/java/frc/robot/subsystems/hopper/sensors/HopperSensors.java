package frc.robot.subsystems.hopper.sensors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intakerollers.IntakeRollersConstants;

public class HopperSensors extends SubsystemBase {

    private CANrange kickerSensor;
    private CANrange upperHopperSensor; 
    private CANrange lowerHopperSensor;
    
    private StatusSignal<Distance> kickerDistance;
    private StatusSignal<Distance> hopperDistance;
    private StatusSignal<Boolean> hopperFull;
    private StatusSignal<Boolean> hopperEmpty;
    private StatusSignal<Boolean> kickerFull;
    private StatusSignal<Boolean> kickerEmpty;

    public HopperSensors() {
        kickerSensor =
            new CANrange(HopperConstants.KICKER_SENSOR_CAN_ID, new CANBus(HopperConstants.SENSORS_CANBUS_NAME));
        kickerSensor.getConfigurator().apply(HopperConstants.KICKER_SENSOR_CONFIGURATION);

        upperHopperSensor =
            new CANrange(HopperConstants.UPPER_HOPPER_CAN_ID, new CANBus(HopperConstants.SENSORS_CANBUS_NAME));
        upperHopperSensor.getConfigurator().apply(HopperConstants.UPPER_HOPPER_CONFIGURATION);
        
        lowerHopperSensor =
            new CANrange(HopperConstants.LOWER_HOPPER_CAN_ID, new CANBus(HopperConstants.SENSORS_CANBUS_NAME));
        lowerHopperSensor.getConfigurator().apply(HopperConstants.LOWER_HOPPER_CONFIGURATION);
    }
}
