package frc.robot.auto;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.shoot.AutoShoot;

public class ShootFuel {
  private RobotContainer robot;

  public ShootFuel(RobotContainer robot) {
    this.robot = robot;
  }

  public Command shootAllFuel() {
    AutoShoot autoShoot = new AutoShoot(robot);

    return Commands.parallel(
            autoShoot,
            robot
                .getHopper()
                .feed()
                .onlyWhile(
                    () -> autoShoot.isReadyToShoot().getAsBoolean() || RobotBase.isSimulation())
                .repeatedly())
        .until(() -> robot.getHopper().isEmpty());
  }

  /**
   * Aligns the shooter then feeds fuel until the hopper is empty.
   *
   * @return the command to execute this sequence
   */
  public Command shootAllFuelStationary() {
    AutoShoot autoShoot = new AutoShoot(robot);

    return Commands.parallel(
            autoShoot,
            Commands.waitUntil(
                    () -> autoShoot.isReadyToShoot().getAsBoolean() || RobotBase.isSimulation())
                .andThen(robot.getHopper().feed().repeatedly()),
            Commands.sequence(
                pulseIntake(),
                Commands.sequence(
                        robot.getIntakeExtension().retract(), robot.getIntakeExtension().extend())
                    .repeatedly()),
            robot.getIntakeRollers().intake())
        .until(() -> robot.getHopper().isEmpty());
  }

  private Command pulseIntake() {
    return Commands.sequence(
        Commands.waitSeconds(2),
        Commands.sequence(
                robot.getIntakeExtension().retractSlow().withTimeout(0.5),
                Commands.waitSeconds(0.25),
                robot.getIntakeExtension().extendSlow().withTimeout(0.5))
            .repeatedly());
  }

  /**
   * Aligns the shooter then feeds fuel. This command does not end on its own.
   *
   * @return the command to execute this sequence
   */
  public Command shoot() {
    AutoShoot autoShoot = new AutoShoot(robot);

    return Commands.parallel(
        autoShoot,
        Commands.waitUntil(
                () -> autoShoot.isReadyToShoot().getAsBoolean() || RobotBase.isSimulation())
            .andThen(robot.getHopper().feed().repeatedly()),
        pulseIntake());
  }

  public Command shootOnTheMove() {
    AutoShoot autoShoot = new AutoShoot(robot);

    return Commands.parallel(
        autoShoot,
        Commands.waitUntil(
                () -> autoShoot.isReadyToShoot().getAsBoolean() || RobotBase.isSimulation())
            .andThen(
                robot
                    .getHopper()
                    .feed()
                    .onlyWhile(
                        () -> autoShoot.isReadyToShoot().getAsBoolean() || RobotBase.isSimulation())
                    .repeatedly()));
  }

  public Command shootAllFuelOnTheMove() {
    AutoShoot autoShoot = new AutoShoot(robot);

    return Commands.parallel(
            autoShoot,
            Commands.waitUntil(
                    () -> autoShoot.isReadyToShoot().getAsBoolean() || RobotBase.isSimulation())
                .andThen(
                    robot
                        .getHopper()
                        .feed()
                        .onlyWhile(
                            () ->
                                autoShoot.isReadyToShoot().getAsBoolean()
                                    || RobotBase.isSimulation())
                        .repeatedly()),
            Commands.sequence(
                pulseIntake(),
                Commands.sequence(
                        robot.getIntakeExtension().retract(), robot.getIntakeExtension().extend())
                    .repeatedly()),
            robot.getIntakeRollers().intake())
        .until(() -> robot.getHopper().isEmpty());
  }
}
