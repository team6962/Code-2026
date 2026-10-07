package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;

import com.team6962.lib.swerve.CommandSwerveDrive;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.shoot.AutoShootConstants;

public class AutoLowerHood {
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);
  private CommandSwerveDrive swerveDrive;
  private Distance HOOD_LOWERING_DISTANCE = Inches.of(30.0);
  private Distance NEAR_OBSTACLES_X = Inches.of(182.11);
  private Distance FAR_OBSTACLES_X = Inches.of(650.12).minus(NEAR_OBSTACLES_X);
  private boolean fineControl = false;

  public AutoLowerHood(CommandSwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public boolean shouldLowerHood() {
    Distance robotX =
        swerveDrive.getPosition2d().plus(AutoShootConstants.shooterTransform).getMeasureX();
    
    if (operator.rightBumper().getAsBoolean()) {
     fineControl = true;
    }
  
    return !(!robotX.isNear(NEAR_OBSTACLES_X, HOOD_LOWERING_DISTANCE)
        && !robotX.isNear(FAR_OBSTACLES_X, HOOD_LOWERING_DISTANCE) 
        && (driver.back().getAsBoolean() 
        || operator.rightTrigger().getAsBoolean()
        || operator.a().getAsBoolean()
        || fineControl));  
  }
}
