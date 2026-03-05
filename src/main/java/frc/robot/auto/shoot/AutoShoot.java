package frc.robot.auto.shoot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.commands.CommandUtil;
import com.team6962.lib.math.AngleMath;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hood.ShooterHood;
import frc.robot.subsystems.shooterrollers.ShooterRollers;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import org.apache.commons.math3.util.Pair;

/** A command that automatically aims and spins up the shooter rollers to shoot at a target. */
public class AutoShoot extends Command {
  public static Translation2d HUB_TRANSLATION =
      new Translation2d(4.62562575, 4.03463125); // Measured with CAD

  /**
   * The swerve drive subsystem, used to get the shooter's pose and velocity for calculating
   * shooting parameters.
   */
  private CommandSwerveDrive swerveDrive;

  /** The turret subsystem, used to aim the shooter in azimuth. */
  private Turret turret;

  /** A filter used to reduce noise in the turret's target velocity measurements. */
  private LinearFilter turretVelocityFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

  /** The hood subsystem, used to aim the shooter in elevation. */
  private ShooterHood hood;

  /** A filter used to reduce noise in the hood's target velocity measurements. */
  private LinearFilter hoodVelocityFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

  /** The shooter rollers subsystem, used to spin up the shooter wheels to the target speed. */
  private ShooterRollers rollers;

  /** The shooter functions, which provide access to shooter calibration data. */
  private ShooterFunctions shooterFunctions;

  /** A supplier that provides the target position. */
  private Supplier<Translation2d> targetSupplier;

  /** The target azimuth angle for the turret. */
  private Angle turretAngleTarget;

  /** The target azimuth velocity of the turret. */
  private AngularVelocity turretVelocityTarget = RotationsPerSecond.of(0);

  /** The target elevation angle for the hood. */
  private Angle hoodAngleTarget;

  /** The target elevation velocity of the hood. */
  private AngularVelocity hoodVelocityTarget = RotationsPerSecond.of(0);

  /** The target angular velocity for the rollers. */
  private AngularVelocity rollerSpeedTarget;

  /** Whether the system is ready to shoot. */
  private boolean readyToShoot;

  /** Whether this command is currently running. */
  private boolean thisCommandRunning = false;

  /** The last time that periodic() was executed. */
  private double previousPeriodicTimestamp = -1.0;

  /**
   * Creates a new AutoShoot command, which automatically aims and spins up the shooter rollers to
   * shoot at a target.
   *
   * @param swerveDrive the swerve drive
   * @param turret the turret
   * @param hood the hood
   * @param rollers the shooter rollers
   * @param targetSupplier a supplier that provides the target position in 3D space
   */
  public AutoShoot(
      CommandSwerveDrive swerveDrive,
      Turret turret,
      ShooterHood hood,
      ShooterRollers rollers,
      ShooterFunctions shooterFunctions,
      Supplier<Translation2d> targetSupplier) {
    this.swerveDrive = swerveDrive;
    this.turret = turret;
    this.hood = hood;
    this.rollers = rollers;
    this.targetSupplier = targetSupplier;
    this.shooterFunctions = shooterFunctions;

    // Create triggers and bind commands to them in order to continuously update
    // subsystem setpoints while this command is running.
    Trigger runningTrigger = new Trigger(() -> thisCommandRunning);

    Command turretCommand = turret.track(() -> turretAngleTarget, () -> turretVelocityTarget);
    Command hoodCommand = hood.track(() -> hoodAngleTarget, () -> hoodVelocityTarget).repeatedly();
    Command rollersCommand = rollers.shoot(() -> rollerSpeedTarget).repeatedly();

    runningTrigger.whileTrue(turretCommand);
    runningTrigger
        .and(() -> CommandUtil.isClearToOverride(hood, hoodCommand))
        .whileTrue(hoodCommand);
    runningTrigger
        .and(() -> CommandUtil.isClearToOverride(rollers, rollersCommand))
        .whileTrue(rollersCommand);
  }

  @Override
  public void initialize() {
    readyToShoot = false;
    turretAngleTarget = null;
    hoodAngleTarget = null;
    rollerSpeedTarget = null;
    turretVelocityTarget = null;
    hoodVelocityTarget = null;
    previousPeriodicTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void end(boolean interrupted) {
    thisCommandRunning = false;
    readyToShoot = false;

    DogLog.log("AutoShoot/Running", false);
    DogLog.log("AutoShoot/ReadyToShoot", readyToShoot);
  }

  /**
   * Returns a trigger that is active when the shooter is ready to shoot.
   *
   * @return a trigger that is active when the shooter is ready to shoot
   */
  public Trigger isReadyToShoot() {
    return new Trigger(() -> readyToShoot);
  }

  /**
   * Predicts the destination of a projectile fired from the shooter given the shooter's pose,
   * velocity, azimuth angle, and hood angle.
   *
   * @param distance the distance to the target if the shooter is stationary
   * @param shooterPose the pose of the shooter
   * @param shooterVelocity the velocity of the shooter
   * @param azimuthAngle the azimuth angle of the shooter
   * @param hoodAngle the hood angle of the shooter
   * @return the predicted destination of the projectile
   */
  private Translation2d predictDestination(
      Distance distance,
      Pose2d shooterPose,
      TranslationalVelocity shooterVelocity,
      Angle azimuthAngle,
      Angle hoodAngle) {

    // Calculate the displacement of the projectile during flight
    Translation2d displacement =
        new Translation2d(distance.in(Meters), new Rotation2d(azimuthAngle));

    // Account for the shooter's initial velocity during flight
    Time flightTime = shooterFunctions.getFlightTime(distance);
    displacement =
        displacement.plus(
            shooterVelocity
                .times(flightTime)
                .times(
                    AutoShootConstants.initialVelocityDisplacementScalarFunction.value(
                        new double[] {distance.in(Inches), hoodAngle.in(Degrees)})));

    // Calculate the final destination of the projectile
    return shooterPose.getTranslation().plus(displacement);
  }

  /**
   * Calculates the static shooting angles (azimuth and hood) required to hit the target from the
   * shooter's pose.
   *
   * @param shooterPose the pose of the shooter
   * @param target the target position
   * @return a pair containing the azimuth and hood angles
   */
  private Pair<Angle, Angle> getStaticShootingAngles(Pose2d shooterPose, Translation2d target) {
    // Calculate the azimuth angle to the target
    Angle azimuthAngle = target.minus(shooterPose.getTranslation()).getAngle().getMeasure();

    // Calculate the hood angle using the hood angle function
    Distance horizontalDistance = Meters.of(shooterPose.getTranslation().getDistance(target));

    Angle hoodAngle = shooterFunctions.getHoodAngle(horizontalDistance);

    return new Pair<Angle, Angle>(azimuthAngle, hoodAngle);
  }

  /**
   * Calculates the moving shooting angles (azimuth and hood) required to hit the target from the
   * shooter's pose and velocity.
   *
   * @param shooterPose the pose of the shooter
   * @param shooterVelocity the velocity of the shooter
   * @param rollersAngularVelocity the angular velocity of the shooter rollers
   * @param target the target position
   * @return a pair containing the azimuth and hood angles
   */
  private Pair<Angle, Angle> getMovingShootingAngles(
      Pose2d shooterPose, TranslationalVelocity shooterVelocity, Translation2d target) {
    Pair<Angle, Angle> angles = getStaticShootingAngles(shooterPose, target);
    Translation2d adjustedTarget = target;

    // Iteratively refine the shooting angles until the predicted destination is
    // within the optimization tolerance of the target.
    for (int i = 0; i < AutoShootConstants.optimizationIterations; i++) {
      // Predict the destination for if the projectile were to be fired with the
      // calculated angles
      Distance distance = Meters.of(shooterPose.getTranslation().getDistance(adjustedTarget));

      Translation2d predictedDestination =
          predictDestination(
              distance, shooterPose, shooterVelocity, angles.getFirst(), angles.getSecond());

      // Adjust the target based on the error between the predicted destination and the
      // actual target
      Translation2d error = predictedDestination.minus(target);
      adjustedTarget = adjustedTarget.minus(error);

      // Calculate the static shooting angles to hit the adjusted target
      angles = getStaticShootingAngles(shooterPose, adjustedTarget);
    }

    return angles;
  }

  private static class ShootingParameters {
    public Angle turretAngle;
    public Angle hoodAngle;
    public AngularVelocity rollerSpeed;

    public ShootingParameters(Angle turretAngle, Angle hoodAngle, AngularVelocity rollerSpeed) {
      this.turretAngle = turretAngle;
      this.hoodAngle = hoodAngle;
      this.rollerSpeed = rollerSpeed;
    }

    public void log(String path) {
      DogLog.log(path + "/TurretAngle", turretAngle.in(Radians), Radians);
      DogLog.log(path + "/HoodAngle", hoodAngle.in(Degrees), Degrees);
      DogLog.log(path + "/RollerSpeed", rollerSpeed.in(RotationsPerSecond), RotationsPerSecond);
    }
  }

  private ShootingParameters calculate(Time poseExtrapolationTime) {
    // Get the target position and initial shooter state
    Translation2d target = targetSupplier.get();

    Twist2d twist = swerveDrive.getArcVelocity();
    twist.dx *= poseExtrapolationTime.in(Seconds);
    twist.dy *= poseExtrapolationTime.in(Seconds);
    twist.dtheta *= poseExtrapolationTime.in(Seconds);
    Pose2d shooterPose =
        swerveDrive.getPosition2d().exp(twist).plus(AutoShootConstants.shooterTransform);
    TranslationalVelocity shooterVelocity = swerveDrive.getTranslationalVelocity();

    // Calculate the ideal shooting angles and roller speed to hit the target
    Pair<Angle, Angle> idealAngles = getMovingShootingAngles(shooterPose, shooterVelocity, target);

    Angle turretAngleTarget = idealAngles.getFirst().minus(shooterPose.getRotation().getMeasure());
    Angle hoodAngleTarget = idealAngles.getSecond();
    AngularVelocity rollerSpeedTarget =
        shooterFunctions.getFlywheelVelocity(
            Meters.of(target.getDistance(shooterPose.getTranslation())));

    return new ShootingParameters(turretAngleTarget, hoodAngleTarget, rollerSpeedTarget);
  }

  @Override
  public void execute() {
    // Get the target position and initial shooter state
    Translation2d target = targetSupplier.get();

    DogLog.log("AutoShoot/TargetX", target.getX());
    DogLog.log("AutoShoot/TargetY", target.getY());

    // Calculate the ideal shooting angles and roller speed to hit the target
    ShootingParameters appliedShootingParameters = calculate(Seconds.of(0.03));
    ShootingParameters currentShootingParameters = calculate(Seconds.of(0));

    appliedShootingParameters.log("AutoShoot/AppliedShootingParameters");
    currentShootingParameters.log("AutoShoot/CurrentShootingParameters");

    Angle previousTurretAngleTarget = turretAngleTarget;
    Angle previousHoodAngleTarget = hoodAngleTarget;

    // Set the target angles and roller speed
    turretAngleTarget = appliedShootingParameters.turretAngle;
    hoodAngleTarget = appliedShootingParameters.hoodAngle;
    rollerSpeedTarget = appliedShootingParameters.rollerSpeed;

    if (previousTurretAngleTarget != null && previousHoodAngleTarget != null) {
      double timeSinceLastPeriodic = Timer.getFPGATimestamp() - previousPeriodicTimestamp;

      turretVelocityTarget =
          RotationsPerSecond.of(
              turretVelocityFilter.calculate(
                  AngleMath.toContinuous(
                          AngleMath.toDiscrete(turretAngleTarget), previousTurretAngleTarget)
                      .minus(previousTurretAngleTarget)
                      .div(Seconds.of(timeSinceLastPeriodic))
                      .in(RotationsPerSecond)));

      hoodVelocityTarget =
          RotationsPerSecond.of(
              hoodVelocityFilter.calculate(
                  hoodAngleTarget
                      .minus(previousHoodAngleTarget)
                      .div(Seconds.of(timeSinceLastPeriodic))
                      .in(RotationsPerSecond)));

      DogLog.log(
          "AutoShoot/AppliedShootingParameters/TurretVelocity",
          turretVelocityTarget.in(RadiansPerSecond),
          RadiansPerSecond);
      DogLog.log(
          "AutoShoot/AppliedShootingParameters/HoodVelocity",
          hoodVelocityTarget.in(DegreesPerSecond),
          DegreesPerSecond);
    }

    previousPeriodicTimestamp = Timer.getFPGATimestamp();

    // Determine if the system is ready to shoot based on whether the shooter is at the target
    // angles
    // and roller speed
    readyToShoot =
        rollers
                .getAngularVelocity()
                .isNear(rollerSpeedTarget, AutoShootConstants.flywheelVelocityTolerance)
            && hood.getPosition().isNear(hoodAngleTarget, AutoShootConstants.hoodAngleTolerance)
            && AngleMath.toContinuous(AngleMath.toDiscrete(turret.getPosition()), turretAngleTarget)
                .isNear(turretAngleTarget, AutoShootConstants.turretAngleTolerance);

    thisCommandRunning = true;

    DogLog.log("AutoShoot/Running", true);
    DogLog.log("AutoShoot/ReadyToShoot", readyToShoot);
  }
}
