package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.beltfloor.BeltFloor;
import frc.robot.subsystems.hopper.kicker.Kicker;
import frc.robot.subsystems.hopper.sensors.HopperSensors;

/**
 * Subsystem for the hopper, which includes the belt floor, kicker, and Sensors, which grabs it from
 * their respective subsystem file
 */
public class Hopper extends SubsystemBase {
  private final BeltFloor beltFloor;
  private final Kicker kicker;
  private final HopperSensors sensors;

  /** Constructor for the Hopper subsystem, which initializes the belt floor, kicker, and sensors */
  public Hopper() {
    this.beltFloor = new BeltFloor();
    this.kicker = new Kicker();
    this.sensors = new HopperSensors();
  }

  /**
   * Command to load the hopper, which runs the belt floor and kicker until the kicker is full
   *
   * @return
   */
  public Command load() {
    return Commands.parallel(beltFloor.feed(), kicker.feed()).until(sensors::isKickerFull);
  }

  /**
   * Command to feed the hopper, which runs the belt floor and kicker for a short time or until the
   * kicker is empty
   *
   * @return
   */
  public Command feed() {
    Command FeedSystem = Commands.parallel(beltFloor.feed(), kicker.feed());
    return Commands.either(
        FeedSystem.withTimeout(0.25),
        FeedSystem.until(sensors::isKickerEmpty),
        sensors::isKickerEmpty);
  }

  /**
   * Command to eject the hopper, which runs the belt floor and kicker in reverse for a short time
   * or until the hopper is empty
   *
   * @return
   */
  public boolean isFull() {
    return sensors.isHopperFull();
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

  // Getters for the belt floor(check BeltFloor.java) for more information.
  public BeltFloor getBeltFloor() {
    return beltFloor;
  }

  // Getters for the belt floor(check Kicker.java) for more information.
  public Kicker getKicker() {
    return kicker;
  }

  // Getters for the belt floor(check hopperSensors.java) for more information.
  public HopperSensors getSensors() {
    return sensors;
  }
}
