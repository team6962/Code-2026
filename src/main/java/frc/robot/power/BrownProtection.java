package frc.robot.power;

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
import frc.robot.subsystems.intakeextension.IntakeExtension;
import frc.robot.subsystems.turret.Turret;

/**
 * Dynamically slows drivetrain and lower-priority mechanism motion when battery voltage drops or
 * robot is drawing too high of a current.
 */
public class BrownProtection extends SubsystemBase {
  private final CommandSwerveDrive swerveDrive;
  private final Turret turret;
  private final ShooterHood shooterHood;
  private final IntakeExtension intakeExtension;

  private final LinearFilter batteryVoltageFilter = LinearFilter.movingAverage(10);
  private final LinearFilter totalCurrentFilter = LinearFilter.movingAverage(10);

  private double startLimitingVoltage = 10.2;
  private double minimumVoltage = 8.8;
  private double startLimitingCurrentAmps = 180.0;
  private double maximumCurrentAmps = 280.0;
  private double minimumDriveScale = 0.45;
  private double minimumMechanismScale = 0.2;

  public BrownProtection(
      CommandSwerveDrive swerveDrive,
      Turret turret,
      ShooterHood shooterHood,
      IntakeExtension intakeExtension) {
    this.swerveDrive = swerveDrive;
    this.turret = turret;
    this.shooterHood = shooterHood;
    this.intakeExtension = intakeExtension;

    DogLog.tunable(
        "Brownout/Start Limiting Voltage",
        startLimitingVoltage,
        value -> startLimitingVoltage = value);
    DogLog.tunable("Brownout/Minimum Voltage", minimumVoltage, value -> minimumVoltage = value);
    DogLog.tunable(
        "Brownout/Start Limiting Current",
        startLimitingCurrentAmps,
        value -> startLimitingCurrentAmps = value);
    DogLog.tunable(
        "Brownout/Maximum Current", maximumCurrentAmps, value -> maximumCurrentAmps = value);
    DogLog.tunable(
        "Brownout/Minimum Drive Scale", minimumDriveScale, value -> minimumDriveScale = value);
    DogLog.tunable(
        "Brownout/Minimum Mechanism Scale",
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
    swerveDrive.setMotionConstraintScale(driveScale);
    turret.setMotionProfileConstraintScale(mechanismScale);
    shooterHood.setMotionProfileConstraintScale(mechanismScale);
    intakeExtension.setMotionProfileConstraintScale(mechanismScale);

    DogLog.log("Brownout/BatteryVoltage", batteryVoltage);
    DogLog.log("Brownout/FilteredBatteryVoltage", filteredBatteryVoltage);
    DogLog.log("Brownout/TotalCurrentAmps", totalCurrentAmps);
    DogLog.log("Brownout/FilteredCurrentAmps", filteredTotalCurrentAmps);
    DogLog.log("Brownout/VoltageBudget", voltageBudget);
    DogLog.log("Brownout/CurrentBudget", currentBudget);
    DogLog.log("Brownout/ProtectionBudget", protectionBudget);
    DogLog.log("Brownout/DriveScale", driveScale);
    DogLog.log("Brownout/MechanismScale", mechanismScale);
  }

  private static double unitRange(double value, double minimumValue, double maximumValue) {
    if (maximumValue <= minimumValue) {
      return value >= maximumValue ? 1.0 : 0.0;
    }

    return MathUtil.clamp((value - minimumValue) / (maximumValue - minimumValue), 0.0, 1.0);
  }
}
