package frc.robot.subsystems.hopper;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.floor.BeltFloor;
import frc.robot.subsystems.hopper.floor.HopperFloor;
import frc.robot.subsystems.hopper.kicker.Kicker;
import frc.robot.subsystems.hopper.sensors.HopperSensors;

/**
 * Subsystem for the hopper, which includes the floor, kicker, and Sensors, which grabs it from
 * their respective subsystem file
 */
public class Hopper extends SubsystemBase {
  private final HopperFloor floor;
  private final Kicker kicker;
  private final HopperSensors sensors;

  private double kickerClearTime = 0.5;
  private double beltFloorPulseTime = 0.2;

  /** Constructor for the Hopper subsystem, which initializes the floor, kicker, and sensors */
  public Hopper() {
    floor = new BeltFloor();
    kicker = new Kicker();
    sensors = new HopperSensors();

    DogLog.tunable(
        "Hopper/Kicker Clear Time (s)",
        kickerClearTime,
        value -> {
          kickerClearTime = value;
        });

    DogLog.tunable(
        "Hopper/Floor Pulse Time (s)",
        beltFloorPulseTime,
        value -> {
          beltFloorPulseTime = value;
        });
  }

  /** Command to dump the hopper, which runs the floor in reverse. */
  public Command dump() {
    return floor.dump();
  }

  /**
   * Command to load the hopper, which runs the floor and kicker until the kicker is full
   *
   * @return
   */
  public Command load() {
    return floor.feed().alongWith(kicker.slowFeed()).until(() -> !sensors.isKickerEmpty());
  }

  /**
   * Command to feed the hopper, which runs the floor and kicker.
   *
   * @return A command that runs the floor and kicker to feed the hopper.
   */
  public Command feed() {
    return Commands.parallel(floor.feed(), kicker.feed());
  }

  /**
   * Attempts to unjam the indexer by running the floor in reverse.
   *
   * @return A command that runs the floor in reverse to attempt to unjam the indexer.
   */
  public Command unjam() {
    return floor.dump();
  }

  /**
   * Command to check if the hopper is empty, which checks the sensors for both the hopper and
   * kicker
   *
   * @return
   */
  public boolean isEmpty() {
    return sensors.isHopperEmpty() && sensors.isKickerEmpty();
  }

  /**
   * Gets the hopper floor subsystem.
   *
   * @return The hopper floor subsystem, which could either be a floor or a roller floor.
   */
  public HopperFloor getFloor() {
    return floor;
  }

  /** Getters for the kicker(check Kicker.java) for more information. */
  public Kicker getKicker() {
    return kicker;
  }

  /** Getters for the hopper sensors(check hopperSensors.java) for more information. */
  public HopperSensors getSensors() {
    return sensors;
  }
}
