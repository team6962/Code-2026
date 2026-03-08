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
}
