package frc.robot.auto.shoot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.apache.commons.math3.util.Pair;

import com.team6962.lib.commands.CommandUtil;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hood.ShooterHood;
import frc.robot.subsystems.shooterrollers.ShooterRollers;
import frc.robot.subsystems.turret.Turret;

/** A command that automatically aims and spins up the shooter rollers to shoot at a target. */
public class AutoShoot extends Command {
  /**
   * The swerve drive subsystem, used to get the shooter's pose and velocity for calculating
   * shooting parameters.
   */
  private CommandSwerveDrive swerveDrive;

  /**
   * The turret subsystem, used to aim the shooter in azimuth.
   */
  private Turret turret;

  /**
   * The hood subsystem, used to aim the shooter in elevation.
   */
  private ShooterHood hood;

  /**
   * The shooter rollers subsystem, used to spin up the shooter wheels to the target speed.
   */
  private ShooterRollers rollers;

  /** The shooter functions, which provide access to shooter calibration data. */
  private ShooterFunctions shooterFunctions;

  /** A supplier that provides the target position. */
  private Supplier<Translation2d> targetSupplier;

  /** The target azimuth angle for the turret. */
  private Angle turretAngleTarget;

  /** The target elevation angle for the hood. */
  private Angle hoodAngleTarget;

  /** The target angular velocity for the rollers. */
  private AngularVelocity rollerSpeedTarget;

  /** Whether the system is ready to shoot. */
  private boolean readyToShoot;

  /** Whether this command is currently running. */
  private boolean thisCommandRunning = false;

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

    Command turretCommand = turret.moveTo(() -> turretAngleTarget).repeatedly();
    Command hoodCommand = hood.moveTo(() -> hoodAngleTarget).repeatedly();
    Command rollersCommand = rollers.shoot(() -> rollerSpeedTarget).repeatedly();

    runningTrigger
        .and(() -> CommandUtil.isClearToOverride(turret, turretCommand))
        .whileTrue(turretCommand);
    runningTrigger
        .and(() -> CommandUtil.isClearToOverride(hood, hoodCommand))
        .whileTrue(hoodCommand);
    runningTrigger
        .and(() -> CommandUtil.isClearToOverride(rollers, rollersCommand))
        .whileTrue(rollersCommand);
  }

  @Override
  public void initialize() {
    thisCommandRunning = true;
    readyToShoot = false;
  }

  @Override
  public void end(boolean interrupted) {
    thisCommandRunning = false;
    readyToShoot = false;
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
    Translation2d displacement = new Translation2d(distance.in(Meters), new Rotation2d(azimuthAngle));

    // Account for the shooter's initial velocity during flight
    Time flightTime = shooterFunctions.getFlightTime(distance);
    displacement =
        displacement.plus(
            shooterVelocity
                .times(flightTime)
                .times(AutoShootConstants.initialVelocityDisplacementScalarFunction.value(new double[] {
                    distance.in(Inches), hoodAngle.in(Degrees)})));

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
  private Pair<Angle, Angle> getStaticShootingAngles(
      Pose2d shooterPose, Translation2d target) {
    // Calculate the azimuth angle to the target
    Angle azimuthAngle =
        shooterPose.getTranslation().minus(target).getAngle().getMeasure();

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
      Pose2d shooterPose,
      TranslationalVelocity shooterVelocity,
      Translation2d target) {
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
              distance,
              shooterPose,
              shooterVelocity,
              angles.getFirst(),
              angles.getSecond());

      // Adjust the target based on the error between the predicted destination and the
      // actual target
      Translation2d error = predictedDestination.minus(target);
      adjustedTarget = target.minus(error);

      // Calculate the static shooting angles to hit the adjusted target
      angles = getStaticShootingAngles(shooterPose, adjustedTarget);
    }

    return angles;
  }

  @Override
  public void execute() {
    // Get the target position and initial shooter state
    Translation2d target = targetSupplier.get();
    Pose2d shooterPose = swerveDrive.getPosition2d().plus(AutoShootConstants.shooterTransform);
    TranslationalVelocity shooterVelocity = swerveDrive.getTranslationalVelocity();

    // Calculate the ideal shooting angles and roller speed to hit the target
    Pair<Angle, Angle> idealAngles = getMovingShootingAngles(shooterPose, shooterVelocity, target);

    // Set the target angles and roller speed
    turretAngleTarget = idealAngles.getFirst().minus(shooterPose.getRotation().getMeasure());
    hoodAngleTarget = idealAngles.getSecond();
    rollerSpeedTarget = shooterFunctions.getFlywheelVelocity(Meters.of(target.getDistance(shooterPose.getTranslation())));

    // Determine if the system is ready to shoot based on whether the shooter is at the target angles
    // and roller speed
    if (!rollers.getAngularVelocity().isNear(rollerSpeedTarget, AutoShootConstants.flywheelVelocityTolerance)) {
      readyToShoot = false;
      return;
    }

    if (!hood.getPosition().isNear(hoodAngleTarget, AutoShootConstants.hoodAngleTolerance)) {
      readyToShoot = false;
      return;
    }

    if (!turret.getPosition().isNear(turretAngleTarget, AutoShootConstants.turretAngleTolerance)) {
      readyToShoot = false;
      return;
    }

    readyToShoot = true;
  }
}
