package frc.robot.controls.shoot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.commands.CommandUtil;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.abstraction.IHood;
import frc.robot.subsystems.abstraction.IShooterRollers;
import frc.robot.subsystems.abstraction.ITurret;
import java.util.function.Supplier;
import org.apache.commons.math3.util.Pair;

/** A command that automatically aims and spins up the shooter rollers to shoot at a target. */
public class AutoShoot extends Command {
  /** The swerve drive, which is used to find the position and velocity of the robot. */
  private CommandSwerveDrive swerveDrive;

  /** The turret that controls the azimuth angle of the shooter. */
  private ITurret turret;

  /** The hood that controls the elevation angle of the shooter. */
  private IHood hood;

  /** The rollers that propel fuel out of the shooter. */
  private IShooterRollers rollers;

  /** A supplier that provides the target position in 3D space. */
  private Supplier<Translation3d> targetSupplier;

  /** The target azimuth angle for the turret. */
  private Angle turretAngleTarget;

  /** The target elevation angle for the hood. */
  private Angle hoodAngleTarget;

  /** The target angular velocity for the rollers. */
  private AngularVelocity rollerSpeedTarget;

  /** Whether the system is ready to shoot. */
  private boolean readyToShoot;

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
      ITurret turret,
      IHood hood,
      IShooterRollers rollers,
      Supplier<Translation3d> targetSupplier) {
    this.swerveDrive = swerveDrive;
    this.turret = turret;
    this.hood = hood;
    this.rollers = rollers;
    this.targetSupplier = targetSupplier;

    // Create triggers and bind commands to them in order to continuously update
    // subsystem setpoints while this command is scheduled.
    Trigger runningTrigger = new Trigger(() -> isScheduled());

    Command turretCommand = turret.moveTo(() -> turretAngleTarget).repeatedly();
    Command hoodCommand = hood.moveTo(() -> hoodAngleTarget).repeatedly();
    Command rollersCommand = rollers.spin(() -> rollerSpeedTarget).repeatedly();

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
   * velocity, azimuth angle, hood angle, roller speed, and target height.
   *
   * @param shooterPose the pose of the shooter
   * @param shooterVelocity the velocity of the shooter
   * @param azimuthAngle the azimuth angle of the shooter
   * @param hoodAngle the hood angle of the shooter
   * @param rollerSpeed the roller speed of the shooter
   * @param targetHeight the height of the target
   * @return the predicted destination of the projectile
   */
  private Translation3d predictDestination(
      Pose2d shooterPose,
      TranslationalVelocity shooterVelocity,
      Angle azimuthAngle,
      Angle hoodAngle,
      AngularVelocity rollerSpeed,
      Distance targetHeight) {
    // Calculate parameters for the shooting model functions
    double[] parameters =
        new double[] {
          hoodAngle.in(Radians), rollerSpeed.in(RadiansPerSecond), targetHeight.in(Meters)
        };

    // Calculate the displacement of the projectile during flight
    double distance = AutoShootConstants.distanceFunction.value(parameters);
    Translation2d displacement = new Translation2d(distance, new Rotation2d(azimuthAngle));

    // Account for the shooter's initial velocity during flight
    Time flightTime = Seconds.of(AutoShootConstants.flightTimeFunction.value(parameters));
    displacement = displacement.plus(shooterVelocity.times(flightTime));

    // Calculate the final destination of the projectile
    Translation2d destination = shooterPose.getTranslation().plus(displacement);

    return new Translation3d(destination.getX(), destination.getY(), targetHeight.in(Meters));
  }

  /**
   * Calculates the static shooting angles (azimuth and hood) required to hit the target from the
   * shooter's pose and velocity.
   *
   * @param shooterPose the pose of the shooter
   * @param shooterVelocity the velocity of the shooter
   * @param target the target position
   * @return a pair containing the azimuth and hood angles
   */
  private Pair<Angle, Angle> getStaticShootingAngles(
      Pose2d shooterPose, AngularVelocity shooterVelocity, Translation3d target) {
    // Calculate the azimuth angle to the target
    Angle azimuthAngle =
        shooterPose.getTranslation().minus(target.toTranslation2d()).getAngle().getMeasure();

    // Calculate the hood angle using the hood angle function
    double horizontalDistance = shooterPose.getTranslation().getDistance(target.toTranslation2d());

    Angle hoodAngle =
        Radians.of(
            AutoShootConstants.hoodAngleFunction.value(
                new double[] {
                  horizontalDistance, shooterVelocity.in(RadiansPerSecond), target.getZ()
                }));

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
      AngularVelocity rollersAngularVelocity,
      Translation3d target) {
    Pair<Angle, Angle> angles;
    Translation3d predictedDestination;
    Translation3d error;
    Translation3d adjustedTarget = target;

    // Iteratively refine the shooting angles until the predicted destination is
    // within the optimization tolerance of the target.
    do {
      // Calculate the static shooting angles to hit the adjusted target
      angles = getStaticShootingAngles(shooterPose, rollersAngularVelocity, adjustedTarget);

      // Predict the destination using the calculated angles
      predictedDestination =
          predictDestination(
              shooterPose,
              shooterVelocity,
              angles.getFirst(),
              angles.getSecond(),
              rollersAngularVelocity,
              Meters.of(target.getZ()));

      // Adjust the target based on the error between the predicted destination and the
      // actual target
      error = predictedDestination.minus(target);
      adjustedTarget = target.minus(error);
    } while (error.getNorm() > AutoShootConstants.optimizationTolerance.in(Meters));

    return angles;
  }

  @Override
  public void execute() {
    // Get the target position and initial shooter state
    Translation3d target = targetSupplier.get();
    Pose2d shooterPose = swerveDrive.getPosition2d().plus(AutoShootConstants.shooterTransform);
    TranslationalVelocity shooterVelocity = swerveDrive.getTranslationalVelocity();

    // Calculate the ideal shooting angles and roller speed to hit the target
    Pair<Angle, Angle> idealAngles =
        getMovingShootingAngles(shooterPose, shooterVelocity, rollers.getVelocity(), target);

    // Set the target angles and roller speed
    turretAngleTarget = idealAngles.getFirst().minus(shooterPose.getRotation().getMeasure());
    hoodAngleTarget = idealAngles.getSecond();
    rollerSpeedTarget =
        RadiansPerSecond.of(
            AutoShootConstants.rollerSpeedFunction.value(
                target.toTranslation2d().getDistance(shooterPose.getTranslation())));

    // Predict the destination for if the projectile were to be fired now
    Translation3d predictedDestination =
        predictDestination(
            shooterPose,
            shooterVelocity,
            turret.getPosition().plus(shooterPose.getRotation().getMeasure()),
            hood.getPosition(),
            rollers.getVelocity(),
            Meters.of(target.getZ()));

    // Calculate the predicted error if the projectile were to be fired now
    double error = predictedDestination.getDistance(target);

    // Determine if the system is ready to shoot based on the error between the
    // predicted destination and the target
    readyToShoot = error <= AutoShootConstants.shootingTolerance.in(Meters);
  }
}
