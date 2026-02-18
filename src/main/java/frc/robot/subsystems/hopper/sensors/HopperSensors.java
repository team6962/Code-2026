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
    /**
     * Constructs a new HopperSensors instance. Initializes the CANrange sensors for the Kicker,
     * Upper Hopper, and Lower Hopper using constants defined in HopperConstants. Applies
     * configuration settings and initializes the StatusSignal objects for distance readings.
     */
    kickerSensor =
        new CANrange(HopperConstants.KICKER_SENSOR_CAN_ID, new CANBus(HopperConstants.CANBUS_NAME));
    kickerSensor.getConfigurator().apply(HopperConstants.KICKER_SENSOR_CONFIGURATION);

    upperHopperSensor =
        new CANrange(HopperConstants.UPPER_HOPPER_CAN_ID, new CANBus(HopperConstants.CANBUS_NAME));
    upperHopperSensor.getConfigurator().apply(HopperConstants.UPPER_HOPPER_CONFIGURATION);

    lowerHopperSensor =
        new CANrange(HopperConstants.LOWER_HOPPER_CAN_ID, new CANBus(HopperConstants.CANBUS_NAME));
    lowerHopperSensor.getConfigurator().apply(HopperConstants.LOWER_HOPPER_CONFIGURATION);

    this.kickerDistance = kickerSensor.getDistance();
    this.upperHopperDistance = upperHopperSensor.getDistance();
    this.lowerHopperDistance = lowerHopperSensor.getDistance();
  }

  /**
   * Gets the current distance measured by the Kicker sensor.
   *
   * @return The distance to the detected object in the Kicker.
   */
  public Distance getKickerDistance() {
    return kickerDistance.getValue();
  }

  /**
   * Gets the current distance measured by the Upper Hopper sensor.
   *
   * @return The distance to the detected object in the Upper Hopper.
   */
  public Distance getUpperHopperDistance() {
    return upperHopperDistance.getValue();
  }

  /**
   * Gets the current distance measured by the Lower Hopper sensor.
   *
   * @return The distance to the detected object in the Lower Hopper.
   */
  public Distance getLowerHopperDistance() {
    return lowerHopperDistance.getValue();
  }

  /**
   * Checks if the Hopper is considered full. Determined by the Upper Hopper sensor detecting an
   * object within the defined threshold.
   *
   * @return {@code true} if the distance is less than UPPER_HOPPER_SENSOR_THRESHOLD indicating an
   *     object is present.
   */
  public boolean isHopperFull() {
    return (upperHopperDistance.getValue().lt(HopperConstants.UPPER_HOPPER_SENSOR_THRESHOLD));
  }

  /**
   * Checks if the Hopper is considered empty. Determined by the Lower Hopper sensor detecting an
   * object within the defined threshold.
   *
   * @return {@code true} if the distance is less than LOWER_HOPPER_SENSOR_THRESHOLD indicating no
   *     object is present.
   */
  public boolean isHopperEmpty() {
    return (lowerHopperDistance.getValue().lt(HopperConstants.LOWER_HOPPER_SENSOR_THRESHOLD));
  }

  /**
   * Checks if the kicker is considered full. Determined by the Kicker sensor detecting an object
   * within the defined threshold.
   *
   * @return {@code true} if the distance is equal than KICKER_SENSOR_FULL_THRESHOLD indicating an
   *     object is present.
   */
  public boolean isKickerFull() {
    return (kickerDistance.getValue().equals(HopperConstants.KICKER_SENSOR_FULL_THRESHOLD));
  }

  /**
   * Checks if the kicker is considered empty. Determined by the Kicker sensor detecting an object
   * within the defined threshold.
   *
   * @return {@code true} if the distance is equal than KICKER_SENSOR_EMPTY_THRESHOLD indicating no
   *     object is present.
   */
  public boolean isKickerEmpty() {
    return (kickerDistance.getValue().equals(HopperConstants.KICKER_SENSOR_EMPTY_THRESHOLD));
  }

  /**
   * Periodically called method to update sensor readings and log values. Refreshes the StatusSignal
   * values for all sensors and logs the current distances and states to DogLog.
   */
  @Override
  public void periodic() {
    StatusUtil.check(
        BaseStatusSignal.refreshAll(kickerDistance, upperHopperDistance, lowerHopperDistance));
    DogLog.log("Hopper/Sensors/KickerDistance", getKickerDistance());
    DogLog.log("Hopper/Sensors/UpperHopperDistance", getUpperHopperDistance());
    DogLog.log("Hopper/Sensors/LowerHopperFull", getLowerHopperDistance());
    DogLog.log("Hopper/Sensors/HopperFull", isHopperFull());
    DogLog.log("Hopper/Sensors/HopperEmpty", isHopperEmpty());
    DogLog.log("Hopper/Sensors/KickerFull", isKickerFull());
    DogLog.log("Hopper/Sensors/KickerEmpty", isKickerEmpty());
  }
}
