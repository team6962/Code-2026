package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Seconds;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.beltfloor.BeltFloor;
import frc.robot.subsystems.hopper.kicker.Kicker;
import frc.robot.subsystems.hopper.sensors.HopperSensors;
import java.util.Set;

/**
 * Subsystem for the hopper, which includes the belt floor, kicker, and Sensors, which grabs it from
 * their respective subsystem file
 */
public class Hopper extends SubsystemBase {
  private final BeltFloor beltFloor;
  private final Kicker kicker;
  private final HopperSensors sensors;

  private double kickerClearTime = 0.5;
  private double beltFloorPulseTime = 0.2;

  /** Constructor for the Hopper subsystem, which initializes the belt floor, kicker, and sensors */
  public Hopper() {
    beltFloor = new BeltFloor();
    kicker = new Kicker();
    sensors = new HopperSensors();

    DogLog.tunable(
        "Hopper/Kicker Clear Time (s)",
        kickerClearTime,
        value -> {
          kickerClearTime = value;
        });

    DogLog.tunable(
        "Hopper/Belt Floor Pulse Time (s)",
        beltFloorPulseTime,
        value -> {
          beltFloorPulseTime = value;
        });
  }

  /** Command to dump the hopper, which runs the belt floor in reverse. */
  public Command dump() {
    return beltFloor.dump();
  }

  /**
   * Command to load the hopper, which runs the belt floor and kicker until the kicker is full
   *
   * @return
   */
  public Command load() {
    return beltFloor.feed().alongWith(kicker.slowFeed()).until(() -> !sensors.isKickerEmpty());
  }

  /**
   * Command to feed the hopper, which runs the belt floor and kicker for a short time or until the
   * kicker is empty
   *
   * @return
   */
  public Command feedSynchronized() {
    return feedSynchronizedWithoutUnjam()
        .withTimeout(Seconds.of(0.5))
        .andThen(
            Commands.either(
                    feedSynchronizedWithoutUnjam().withTimeout(Seconds.of(0.5)),
                    unjamDuringSynchronizedFeed(),
                    () -> sensors.isFeedingSuccessfully() || sensors.isKickerEmpty())
                .repeatedly());
  }

  private Command feedSynchronizedWithoutUnjam() {
    return Commands.parallel(beltFloor.feed(), kicker.feed())
        .deadlineFor(
            Commands.startEnd(
                () -> DogLog.log("Hopper/FeedState", "Sync"),
                () -> DogLog.log("Hopper/FeedState", "Off")));
  }

  private Command unjamDuringSynchronizedFeed() {
    return Commands.defer(
        () ->
            Commands.sequence(
                    beltFloor
                        .reverse()
                        .alongWith(kicker.feed())
                        .until(sensors::isKickerEmpty)
                        .withTimeout(kickerClearTime))
                .deadlineFor(
                    Commands.startEnd(
                        () -> DogLog.log("Hopper/FeedState", "Unjam"),
                        () -> DogLog.log("Hopper/FeedState", "Off"))),
        Set.of(beltFloor, kicker));
  }

  /**
   * Command to feed the hopper, which runs the belt floor and kicker in a pulsing pattern, feeding
   * a new row of fuel into the queuer, then emptying the kicker/queuer before feeding in the next
   * row.
   *
   * @return A command that feeds fuel from the hopper to the shooter by pulsing the kicker and belt
   *     floor out of sync, which is less likely to get jammed than feedSynchronized.
   */
  public Command feedPulsing() {
    return Commands.repeatingSequence(
        kicker.feed().alongWith(beltFloor.slowReverse()).until(sensors::isKickerEmpty),
        beltFloor
            .feed()
            .withDeadline(
                Commands.parallel(
                    Commands.defer(() -> Commands.waitSeconds(beltFloorPulseTime), Set.of()),
                    Commands.waitUntil(() -> !sensors.isKickerEmpty())))
            .deadlineFor(
                Commands.startEnd(
                    () -> DogLog.log("Hopper/FeedState", "Pulsing"),
                    () -> DogLog.log("Hopper/FeedState", "Off"))));
  }

  /**
   * Attempts to unjam the indexer by running the belt floor in reverse.
   *
   * @return A command that runs the belt floor in reverse to attempt to unjam the indexer.
   */
  public Command unjam() {
    return beltFloor.dump();
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

  /** Getters for the belt floor(check BeltFloor.java) for more information. */
  public BeltFloor getBeltFloor() {
    return beltFloor;
  }

  /** Getters for the kicker(check Kicker.java) for more information. */
  public Kicker getKicker() {
    return kicker;
  }

  /** Getters for the hopper sensors(check hopperSensors.java) for more information. */
  public HopperSensors getSensors() {
    return sensors;
  }

  @Override
  public void periodic() {}
}
