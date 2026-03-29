package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.team6962.lib.logging.CurrentDrawLogger;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.ShooterHood;
import frc.robot.subsystems.hopper.beltfloor.BeltFloor;
import frc.robot.subsystems.intakeextension.IntakeExtension;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooterrollers.ShooterRollers;
import frc.robot.subsystems.turret.Turret;

/**
 * Dynamically slows subsystems when electrical headroom gets tight, prioritizing shooter/feed while
 * shooting, intake while intaking, and drivetrain otherwise.
 *
 * Drive limits are applied as hard velocity caps directly in {@link XBoxTeleopSwerveCommand} so
 * autonomous paths are unaffected
 */
public class BrownoutProtection extends SubsystemBase {
  private static final int ELECTRICAL_SAMPLE_WINDOW = 8;
  private static final double ACTIVE_MECHANISM_VOLTAGE_THRESHOLD_VOLTS = 1.0;
  private static final double ACTIVE_SHOOTER_SPEED_THRESHOLD_RPS = 5.0;

  private final CommandSwerveDrive swerveDrive;
  private final XBoxTeleopSwerveCommand teleopSwerveCommand;
  private final ShooterRollers shooterRollers;
  private final Turret turret;
  private final ShooterHood shooterHood;
  private final IntakeExtension intakeExtension;
  private final BeltFloor beltFloor;
  private final IntakeRollers intakeRollers;

  private final double[] recentBatteryVoltageSamplesVolts = new double[ELECTRICAL_SAMPLE_WINDOW];
  private final double[] recentTotalCurrentSamplesAmps = new double[ELECTRICAL_SAMPLE_WINDOW];
  private int recentElectricalSampleCount = 0;
  private int nextElectricalSampleIndex = 0;

  private double startLimitingVoltage = 10.2;
  private double minimumVoltage = 8.8;
  private double startLimitingCurrentAmps = 180.0;
  private double maximumCurrentAmps = 280.0;
  private double minimumDrivePriorityVelocityScalingFactor = 0.45;
  private double minimumReducedDriveVelocityScalingFactor = 0.3;
  private double minimumPriorityMechanismScalingFactor = 0.65;
  private double minimumReducedMechanismScalingFactor = 0.2;

  private enum PriorityMode {
    DRIVING,
    INTAKING,
    SHOOTING
  }

  public BrownoutProtection(
      CommandSwerveDrive swerveDrive,
      XBoxTeleopSwerveCommand teleopSwerveCommand,
      ShooterRollers shooterRollers,
      Turret turret,
      ShooterHood shooterHood,
      IntakeExtension intakeExtension,
      BeltFloor beltFloor,
      IntakeRollers intakeRollers) {
    this.swerveDrive = swerveDrive;
    this.teleopSwerveCommand = teleopSwerveCommand;
    this.shooterRollers = shooterRollers;
    this.turret = turret;
    this.shooterHood = shooterHood;
    this.intakeExtension = intakeExtension;
    this.beltFloor = beltFloor;
    this.intakeRollers = intakeRollers;

    DogLog.tunable(
        "BrownoutProtection/Start Limiting Voltage",
        startLimitingVoltage,
        value -> startLimitingVoltage = value);
    DogLog.tunable(
        "BrownoutProtection/Minimum Voltage", minimumVoltage, value -> minimumVoltage = value);
    DogLog.tunable(
        "BrownoutProtection/Start Limiting Current",
        startLimitingCurrentAmps,
        value -> startLimitingCurrentAmps = value);
    DogLog.tunable(
        "BrownoutProtection/Maximum Current",
        maximumCurrentAmps,
        value -> maximumCurrentAmps = value);
    DogLog.tunable(
        "BrownoutProtection/Minimum Drive Priority Velocity Scaling Factor",
        minimumDrivePriorityVelocityScalingFactor,
        value -> minimumDrivePriorityVelocityScalingFactor = value);
    DogLog.tunable(
        "BrownoutProtection/Minimum Reduced Drive Velocity Scaling Factor",
        minimumReducedDriveVelocityScalingFactor,
        value -> minimumReducedDriveVelocityScalingFactor = value);
    DogLog.tunable(
        "BrownoutProtection/Minimum Priority Mechanism Scaling Factor",
        minimumPriorityMechanismScalingFactor,
        value -> minimumPriorityMechanismScalingFactor = value);
    DogLog.tunable(
        "BrownoutProtection/Minimum Reduced Mechanism Scaling Factor",
        minimumReducedMechanismScalingFactor,
        value -> minimumReducedMechanismScalingFactor = value);
  }

  @Override
  public void periodic() {
    double batteryVoltageVolts = RobotController.getBatteryVoltage();
    double totalCurrentAmps = CurrentDrawLogger.getTotalCurrent().in(Amps);

    recordElectricalSample(batteryVoltageVolts, totalCurrentAmps);

    double recentMinimumBatteryVoltageVolts = getRecentMinimumBatteryVoltageVolts();
    double recentMaximumTotalCurrentAmps = getRecentMaximumTotalCurrentAmps();

    double voltageHeadroomFraction =
        mapToUnitRange(recentMinimumBatteryVoltageVolts, minimumVoltage, startLimitingVoltage);
    double currentHeadroomFraction =
        1.0
            - mapToUnitRange(
                recentMaximumTotalCurrentAmps, startLimitingCurrentAmps, maximumCurrentAmps);
    // 1.0 means there is enough electrical headroom for full performance. 0.0 means outputs
    // should be clamped to their configured minimum scaling factors.
    double allowedPerformanceFraction =
        RobotState.isDisabled() ? 1.0 : Math.min(voltageHeadroomFraction, currentHeadroomFraction);

    PriorityMode priorityMode = determinePriorityMode();

    double prioritizedDriveVelocityScalingFactor =
        MathUtil.interpolate(
            minimumDrivePriorityVelocityScalingFactor, 1.0, allowedPerformanceFraction);
    double reducedDriveVelocityScalingFactor =
        MathUtil.interpolate(
            minimumReducedDriveVelocityScalingFactor, 1.0, allowedPerformanceFraction);
    double prioritizedMechanismScalingFactor =
        MathUtil.interpolate(
            minimumPriorityMechanismScalingFactor, 1.0, allowedPerformanceFraction);
    double reducedMechanismScalingFactor =
        MathUtil.interpolate(minimumReducedMechanismScalingFactor, 1.0, allowedPerformanceFraction);

    double driveVelocityScalingFactor =
        priorityMode == PriorityMode.DRIVING
            ? prioritizedDriveVelocityScalingFactor
            : reducedDriveVelocityScalingFactor;
    double shooterAimingScalingFactor =
        priorityMode == PriorityMode.SHOOTING
            ? prioritizedMechanismScalingFactor
            : reducedMechanismScalingFactor;
    double intakeMotionScalingFactor =
        priorityMode == PriorityMode.INTAKING
            ? prioritizedMechanismScalingFactor
            : reducedMechanismScalingFactor;
    double beltFloorVoltageScalingFactor =
        priorityMode == PriorityMode.DRIVING
            ? reducedMechanismScalingFactor
            : prioritizedMechanismScalingFactor;
    double intakeRollerVoltageScalingFactor =
        priorityMode == PriorityMode.INTAKING
            ? prioritizedMechanismScalingFactor
            : reducedMechanismScalingFactor;

    // Apply drive limits as hard velocity caps in teleop command only
    // Auton paths go through PathPlanner/VelocityMotion, are never affected
    double maxLinearVelocityMetersPerSecond =
        swerveDrive.getConstants().Driving.MaxLinearVelocity.in(MetersPerSecond);
    double maxAngularVelocityRadiansPerSecond =
        swerveDrive.getConstants().Driving.MaxAngularVelocity.in(RadiansPerSecond);
    teleopSwerveCommand.addDynamicVelocityLimits(
        MetersPerSecond.of(driveVelocityScalingFactor * maxLinearVelocityMetersPerSecond),
        RadiansPerSecond.of(driveVelocityScalingFactor * maxAngularVelocityRadiansPerSecond));

    turret.setMotionProfileConstraintScale(shooterAimingScalingFactor);
    shooterHood.setMotionProfileConstraintScale(shooterAimingScalingFactor);
    intakeExtension.setMotionProfileConstraintScale(intakeMotionScalingFactor);
    beltFloor.setVoltageScale(beltFloorVoltageScalingFactor);
    intakeRollers.setVoltageScale(intakeRollerVoltageScalingFactor);

    DogLog.log("BrownoutProtection/PriorityMode", priorityMode.name());
    DogLog.log("BrownoutProtection/BatteryVoltageVolts", batteryVoltageVolts);
    DogLog.log(
        "BrownoutProtection/RecentMinimumBatteryVoltageVolts", recentMinimumBatteryVoltageVolts);
    DogLog.log("BrownoutProtection/TotalCurrentAmps", totalCurrentAmps);
    DogLog.log("BrownoutProtection/RecentMaximumTotalCurrentAmps", recentMaximumTotalCurrentAmps);
    DogLog.log("BrownoutProtection/VoltageHeadroomFraction", voltageHeadroomFraction);
    DogLog.log("BrownoutProtection/CurrentHeadroomFraction", currentHeadroomFraction);
    DogLog.log("BrownoutProtection/AllowedPerformanceFraction", allowedPerformanceFraction);
    DogLog.log("BrownoutProtection/DriveVelocityScalingFactor", driveVelocityScalingFactor);
    DogLog.log("BrownoutProtection/ShooterAimingScalingFactor", shooterAimingScalingFactor);
    DogLog.log("BrownoutProtection/IntakeMotionScalingFactor", intakeMotionScalingFactor);
    DogLog.log("BrownoutProtection/BeltFloorVoltageScalingFactor", beltFloorVoltageScalingFactor);
    DogLog.log(
        "BrownoutProtection/IntakeRollerVoltageScalingFactor", intakeRollerVoltageScalingFactor);
  }

  private PriorityMode determinePriorityMode() {
    boolean shooterActive =
        shooterRollers.getCurrentCommand() != null
            || Math.abs(shooterRollers.getMotorVoltage().in(Volts))
                > ACTIVE_MECHANISM_VOLTAGE_THRESHOLD_VOLTS
            || Math.abs(shooterRollers.getAngularVelocity().in(RotationsPerSecond))
                > ACTIVE_SHOOTER_SPEED_THRESHOLD_RPS;

    if (shooterActive) {
      return PriorityMode.SHOOTING;
    }

    boolean intakeActive =
        intakeExtension.getCurrentCommand() != null
            || intakeRollers.getAppliedVoltage().in(Volts)
                > ACTIVE_MECHANISM_VOLTAGE_THRESHOLD_VOLTS
            || beltFloor.getMotorVoltage().in(Volts) > ACTIVE_MECHANISM_VOLTAGE_THRESHOLD_VOLTS;

    return intakeActive ? PriorityMode.INTAKING : PriorityMode.DRIVING;
  }

  private void recordElectricalSample(double batteryVoltageVolts, double totalCurrentAmps) {
    recentBatteryVoltageSamplesVolts[nextElectricalSampleIndex] = batteryVoltageVolts;
    recentTotalCurrentSamplesAmps[nextElectricalSampleIndex] = totalCurrentAmps;
    nextElectricalSampleIndex = (nextElectricalSampleIndex + 1) % ELECTRICAL_SAMPLE_WINDOW;
    recentElectricalSampleCount =
        Math.min(recentElectricalSampleCount + 1, ELECTRICAL_SAMPLE_WINDOW);
  }

  private double getRecentMinimumBatteryVoltageVolts() {
    double recentMinimumBatteryVoltageVolts = Double.POSITIVE_INFINITY;

    for (int i = 0; i < recentElectricalSampleCount; i++) {
      recentMinimumBatteryVoltageVolts =
          Math.min(recentMinimumBatteryVoltageVolts, recentBatteryVoltageSamplesVolts[i]);
    }

    return recentElectricalSampleCount == 0
        ? startLimitingVoltage
        : recentMinimumBatteryVoltageVolts;
  }

  private double getRecentMaximumTotalCurrentAmps() {
    double recentMaximumTotalCurrentAmps = Double.NEGATIVE_INFINITY;

    for (int i = 0; i < recentElectricalSampleCount; i++) {
      recentMaximumTotalCurrentAmps =
          Math.max(recentMaximumTotalCurrentAmps, recentTotalCurrentSamplesAmps[i]);
    }

    return recentElectricalSampleCount == 0 ? 0.0 : recentMaximumTotalCurrentAmps;
  }

  /**
   * Maps a value into the inclusive unit interval, where {@code minimumValue} becomes {@code 0.0}
   * and {@code maximumValue} becomes {@code 1.0}.
   */
  private static double mapToUnitRange(double value, double minimumValue, double maximumValue) {
    if (maximumValue <= minimumValue) {
      return value >= maximumValue ? 1.0 : 0.0;
    }

    return MathUtil.clamp((value - minimumValue) / (maximumValue - minimumValue), 0.0, 1.0);
  }
}