package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.swerve.CommandSwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class AutoLowerHood {
  private CommandSwerveDrive swerveDrive;
  private Distance HOOD_LOWERING_THRESHOLD = Meters.of(0.25);

  public AutoLowerHood(CommandSwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public boolean shouldLowerHood() {
    Translation2d robotPosition = swerveDrive.getPosition2d().getTranslation();
    Distance leftTrenchDistance =
        Meters.of(robotPosition.getDistance(FieldPositions.Trench.LEFT_CENTER));
    Distance rightTrenchDistance =
        Meters.of(robotPosition.getDistance(FieldPositions.Trench.RIGHT_CENTER));
    if (leftTrenchDistance.lt(HOOD_LOWERING_THRESHOLD)
        || rightTrenchDistance.lt(HOOD_LOWERING_THRESHOLD)) {
      return true;
    } else {
      return false;
    }
  }
  ;
}
