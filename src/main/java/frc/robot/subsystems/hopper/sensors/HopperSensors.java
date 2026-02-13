package frc.robot.subsystems.hopper.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.team6962.lib.phoenix.StatusUtil;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;

public class HopperSensors extends SubsystemBase {

    private CANrange kickerSensor;
    private CANrange upperHopperSensor; 
    private CANrange lowerHopperSensor;
    
    private StatusSignal<Distance> kickerDistance;
    private StatusSignal<Distance> upperHopperDistance;
    private StatusSignal<Distance> lowerHopperDistance;

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

        this.kickerDistance = kickerSensor.getDistance();
        this.upperHopperDistance = upperHopperSensor.getDistance();
        this.lowerHopperDistance = lowerHopperSensor.getDistance();
    }

    public Distance getKickerDistance() {
        return kickerDistance.getValue();
    }

    public Distance getUpperHopperDistance() {
        return upperHopperDistance.getValue();
    }

    public Distance getLowerHopperDistance() {
        return lowerHopperDistance.getValue();
    }

    public boolean isHopperFull() {   
        if (upperHopperDistance.getValue().lt(HopperConstants.UPPER_HOPPER_SENSOR_THRESHOLD)) {
            return true; 
        }
        return false;
    }

    public boolean isHopperEmpty() {   
        if (lowerHopperDistance.getValue().lt(HopperConstants.LOWER_HOPPER_SENSOR_THRESHOLD)) {
            return true; 
        }
        return false;
    }

    public boolean isKickerFull() {   
        if (kickerDistance.getValue().lt(HopperConstants.KICKER_SENSOR_THRESHOLD)) {
            return true; 
        }
        return false;
    }

    public boolean isKickerEmpty() {   
        if (kickerDistance.getValue().gt(HopperConstants.KICKER_SENSOR_THRESHOLD)) {
            return true; 
        }
        return false;
    }

  @Override
  public void periodic() {
    StatusUtil.check(
        BaseStatusSignal.refreshAll(kickerDistance));
    DogLog.log("Hopper/Sensors/KickerDistance", getKickerDistance());
    DogLog.log("Hopper/Sensors/UpperHopperDistance", getUpperHopperDistance());
    DogLog.log("Hopper/Sensors/LowerHopperFull", getLowerHopperDistance());
    DogLog.log("Hopper/Sensors/HopperFull", isHopperFull());
    DogLog.log("Hopper/Sensors/HopperEmpty", isHopperEmpty());
    DogLog.log("Hopper/Sensors/KickerFull", isKickerFull());
    DogLog.log("Hopper/Sensors/KickerEmpty", isKickerEmpty());
  }
}
