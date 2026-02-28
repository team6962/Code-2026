package frc.robot.subsystems.hopper.sensors;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.team6962.lib.phoenix.StatusUtil;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;

/**
 * Uses three CANrange distance sensors in the hopper and kicker to determine how much fuel is in
 * the robot and where the fuel is.
 */
public class HopperSensors extends SubsystemBase {
  private CANrange kickerSensor;
  private CANrange lowerHopperSensor;

  private StatusSignal<Distance> kickerDistance;
  private StatusSignal<Distance> lowerHopperDistance;

  private Distance upperHopperSensorFullThreshold =
      HopperConstants.UPPER_HOPPER_SENSOR_FULL_THRESHOLD;
  private Distance lowerHopperSensorEmptyThreshold =
      HopperConstants.LOWER_HOPPER_SENSOR_EMPTY_THRESHOLD;
  private Distance kickerSensorFullThreshold = HopperConstants.KICKER_SENSOR_FULL_THRESHOLD;
  private Distance kickerSensorEmptyThreshold = HopperConstants.KICKER_SENSOR_EMPTY_THRESHOLD;

  private double lastKickerNotFullTimestamp;
  private double lastKickerFullTimetstamp;

  private double jamTimeWhenFull = 0.25;
  private double jamTimeWhenEmpty = 0.25;

  /**
   * Constructs a new HopperSensors instance. Initializes the CANrange sensors for the Kicker, Upper
   * Hopper, and Lower Hopper using constants defined in HopperConstants. Applies configuration
   * settings and initializes the StatusSignal objects for distance readings.
   */
  public HopperSensors() {
    kickerSensor =
        new CANrange(HopperConstants.KICKER_SENSOR_CAN_ID, new CANBus(HopperConstants.CANBUS_NAME));
    kickerSensor.getConfigurator().apply(HopperConstants.KICKER_SENSOR_CONFIGURATION);

    lowerHopperSensor =
        new CANrange(HopperConstants.LOWER_HOPPER_CAN_ID, new CANBus(HopperConstants.CANBUS_NAME));
    lowerHopperSensor.getConfigurator().apply(HopperConstants.LOWER_HOPPER_CONFIGURATION);

    this.kickerDistance = kickerSensor.getDistance();
    this.lowerHopperDistance = lowerHopperSensor.getDistance();

    DogLog.tunable(
        "HopperSensors/Hopper Full Threshold (in)",
        upperHopperSensorFullThreshold.in(Inches),
        newValue -> {
          upperHopperSensorFullThreshold = Inches.of(newValue);
        });

    DogLog.tunable(
        "HopperSensors/Hopper Empty Threshold (in)",
        lowerHopperSensorEmptyThreshold.in(Inches),
        newValue -> {
          lowerHopperSensorEmptyThreshold = Inches.of(newValue);
        });

    DogLog.tunable(
        "HopperSensors/Kicker Full Threshold (in)",
        kickerSensorFullThreshold.in(Inches),
        newValue -> {
          kickerSensorFullThreshold = Inches.of(newValue);
        });

    DogLog.tunable(
        "HopperSensors/Kicker Empty Threshold (in)",
        kickerSensorEmptyThreshold.in(Inches),
        newValue -> {
          kickerSensorEmptyThreshold = Inches.of(newValue);
        });
    
    DogLog.tunable(
      "HopperSensors/Jam Time When Full (s)", jamTimeWhenFull, value -> {
        jamTimeWhenFull = value;
      });
    
    DogLog.tunable(
      "HopperSensors/Jam Time When Empty (s)", jamTimeWhenEmpty, value -> {
        jamTimeWhenEmpty = value;
      });
  }

  /**
   * Gets the current distance measured by the Kicker sensor.
   *
   * @return The distance to the detected object in the Kicker.
   */
  private Distance getKickerDistance() {
    return kickerDistance.getValue();
  }

  /**
   * Gets the current distance measured by the Lower Hopper sensor.
   *
   * @return The distance to the detected object in the Lower Hopper.
   */
  private Distance getLowerHopperDistance() {
    return lowerHopperDistance.getValue();
  }

  /**
   * Checks if the Hopper is considered empty. Determined by the Lower Hopper sensor detecting an
   * object within the defined threshold.
   *
   * @return {@code true} if the distance is less than LOWER_HOPPER_SENSOR_THRESHOLD indicating no
   *     object is present.
   */
  public boolean isHopperEmpty() {
    return lowerHopperDistance.getValue().gt(lowerHopperSensorEmptyThreshold);
  }

  /**
   * Checks if the kicker is considered full. Determined by the Kicker sensor detecting an object
   * within the defined threshold.
   *
   * @return {@code true} if the distance is equal than KICKER_SENSOR_FULL_THRESHOLD indicating an
   *     object is present.
   */
  public boolean isKickerFull() {
    return kickerDistance.getValue().lt(kickerSensorFullThreshold);
  }

  /**
   * Checks if the kicker is considered empty. Determined by the Kicker sensor detecting an object
   * within the defined threshold.
   *
   * @return {@code true} if the distance is equal than KICKER_SENSOR_EMPTY_THRESHOLD indicating no
   *     object is present.
   */
  public boolean isKickerEmpty() {
    return kickerDistance.getValue().gt(kickerSensorEmptyThreshold);
  }

  public boolean isFeedingSuccessfully() {
    return isKickerFull()
        ? Timer.getFPGATimestamp() < lastKickerNotFullTimestamp + jamTimeWhenFull
        : Timer.getFPGATimestamp() < lastKickerFullTimetstamp + jamTimeWhenEmpty;
  }

  /**
   * Periodically called method to update sensor readings and log values. Refreshes the StatusSignal
   * values for all sensors and logs the current distances and states to DogLog.
   */
  @Override
  public void periodic() {
    StatusUtil.check(BaseStatusSignal.refreshAll(kickerDistance, lowerHopperDistance));

    if (isKickerFull()) {
      lastKickerFullTimetstamp = Timer.getFPGATimestamp();
    } else {
      lastKickerNotFullTimestamp = Timer.getFPGATimestamp();
    }

    if (isKickerEmpty()) {
      lastKickerFullTimetstamp = Timer.getFPGATimestamp();
      lastKickerNotFullTimestamp = Timer.getFPGATimestamp();
    }

    DogLog.log("Hopper/Sensors/KickerDistance", getKickerDistance());
    DogLog.log("Hopper/Sensors/LowerHopperDistance", getLowerHopperDistance());
    DogLog.log("Hopper/Sensors/HopperEmpty", isHopperEmpty());
    DogLog.log("Hopper/Sensors/KickerFull", isKickerFull());
    DogLog.log("Hopper/Sensors/KickerEmpty", isKickerEmpty());
    DogLog.log("Hopper/Sensors/FeedingSuccessfully", isFeedingSuccessfully());
  }
}
