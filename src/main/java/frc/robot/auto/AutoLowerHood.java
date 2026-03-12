package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;

import com.team6962.lib.swerve.CommandSwerveDrive;
import edu.wpi.first.units.measure.Distance;
import frc.robot.auto.shoot.AutoShootConstants;

public class AutoLowerHood {
  private CommandSwerveDrive swerveDrive;
  private Distance HOOD_LOWERING_DISTANCE = Inches.of(30.0);
  private Distance NEAR_OBSTACLES_X = Inches.of(182.11);
  private Distance FAR_OBSTACLES_X = Inches.of(650.12).minus(NEAR_OBSTACLES_X);

  public AutoLowerHood(CommandSwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public boolean shouldLowerHood() {
    Distance robotX =
        swerveDrive.getPosition2d().plus(AutoShootConstants.shooterTransform).getMeasureX();

    if (robotX.isNear(NEAR_OBSTACLES_X, HOOD_LOWERING_DISTANCE)
        || robotX.isNear(FAR_OBSTACLES_X, HOOD_LOWERING_DISTANCE)) {
      return true;
    } else {
      return false;
    }
  }
}
