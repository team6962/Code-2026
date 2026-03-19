package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.team6962.lib.logging.CurrentDrawLogger;
import com.team6962.lib.swerve.CommandSwerveDrive;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.ShooterHood;
import frc.robot.subsystems.hopper.beltfloor.BeltFloor;
import frc.robot.subsystems.intakeextension.IntakeExtension;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.turret.Turret;

/**
 * Dynamically slows drivetrain and lower-priority mechanism motion when battery voltage drops or
 * robot is drawing too high of a current.
 */
public class BrownoutProtection extends SubsystemBase {
  private final CommandSwerveDrive swerveDrive;
  private final Turret turret;
  private final ShooterHood shooterHood;
  private final IntakeExtension intakeExtension;
  private final BeltFloor beltFloor;
  private final IntakeRollers intakeRollers;

  private final LinearFilter batteryVoltageFilter = LinearFilter.movingAverage(10);
  private final LinearFilter totalCurrentFilter = LinearFilter.movingAverage(10);

  private double startLimitingVoltage = 10.2;
  private double minimumVoltage = 8.8;
  private double startLimitingCurrentAmps = 180.0;
  private double maximumCurrentAmps = 280.0;
  private double minimumDriveScale = 0.45;
  private double minimumMechanismScale = 0.2;

  public BrownoutProtection(
      CommandSwerveDrive swerveDrive,
      Turret turret,
      ShooterHood shooterHood,
      IntakeExtension intakeExtension,
      BeltFloor beltFloor,
      IntakeRollers intakeRollers) {
    this.swerveDrive = swerveDrive;
    this.turret = turret;
    this.shooterHood = shooterHood;
    this.intakeExtension = intakeExtension;
    this.beltFloor = beltFloor;
    this.intakeRollers = intakeRollers;

    DogLog.tunable(
        "BrownoutProtection/Start Limiting Voltage",
        startLimitingVoltage,
        value -> startLimitingVoltage = value);
    DogLog.tunable("BrownoutProtection/Minimum Voltage", minimumVoltage, value -> minimumVoltage = value);
    DogLog.tunable(
        "BrownoutProtection/Start Limiting Current",
        startLimitingCurrentAmps,
        value -> startLimitingCurrentAmps = value);
    DogLog.tunable(
        "BrownoutProtection/Maximum Current", maximumCurrentAmps, value -> maximumCurrentAmps = value);
    DogLog.tunable(
        "BrownoutProtection/Minimum Drive Scale", minimumDriveScale, value -> minimumDriveScale = value);
    DogLog.tunable(
        "BrownoutProtection/Minimum Mechanism Scale",
        minimumMechanismScale,
        value -> minimumMechanismScale = value);
  }

  @Override
  public void periodic() {
    double batteryVoltage = RobotController.getBatteryVoltage();
    double filteredBatteryVoltage = batteryVoltageFilter.calculate(batteryVoltage);
    double totalCurrentAmps = CurrentDrawLogger.getTotalCurrent().in(Amps);
    double filteredTotalCurrentAmps = totalCurrentFilter.calculate(totalCurrentAmps);

    double voltageBudget = unitRange(filteredBatteryVoltage, minimumVoltage, startLimitingVoltage);
    double currentBudget =
        1.0 - unitRange(filteredTotalCurrentAmps, startLimitingCurrentAmps, maximumCurrentAmps);
    double protectionBudget =
        RobotState.isDisabled() ? 1.0 : Math.min(voltageBudget, currentBudget);

    double driveScale = MathUtil.interpolate(minimumDriveScale, 1.0, protectionBudget);
    double mechanismScale = MathUtil.interpolate(minimumMechanismScale, 1.0, protectionBudget);

    swerveDrive.setVelocityScale(driveScale);
    turret.setMotionProfileConstraintScale(mechanismScale);
    shooterHood.setMotionProfileConstraintScale(mechanismScale);
    intakeExtension.setMotionProfileConstraintScale(mechanismScale);
    beltFloor.setVoltageScale(mechanismScale);
    intakeRollers.setVoltageScale(mechanismScale);

    DogLog.log("BrownoutProtection/BatteryVoltage", batteryVoltage);
    DogLog.log("BrownoutProtection/FilteredBatteryVoltage", filteredBatteryVoltage);
    DogLog.log("BrownoutProtection/TotalCurrentAmps", totalCurrentAmps);
    DogLog.log("BrownoutProtection/FilteredCurrentAmps", filteredTotalCurrentAmps);
    DogLog.log("BrownoutProtection/VoltageBudget", voltageBudget);
    DogLog.log("BrownoutProtection/CurrentBudget", currentBudget);
    DogLog.log("BrownoutProtection/ProtectionBudget", protectionBudget);
    DogLog.log("BrownoutProtection/DriveScale", driveScale);
    DogLog.log("BrownoutProtection/MechanismScale", mechanismScale);
  }

  private static double unitRange(double value, double minimumValue, double maximumValue) {
    if (maximumValue <= minimumValue) {
      return value >= maximumValue ? 1.0 : 0.0;
    }

    return MathUtil.clamp((value - minimumValue) / (maximumValue - minimumValue), 0.0, 1.0);
  }
}
