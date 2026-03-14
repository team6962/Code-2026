package com.team6962.lib.swerve.motion;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team6962.lib.math.SwerveKinematicsUtil;
import com.team6962.lib.phoenix.control.PositionControlRequest;
import com.team6962.lib.swerve.MotionSwerveDrive;
import com.team6962.lib.swerve.config.SteerMotorConstants;
import com.team6962.lib.swerve.module.SwerveModule;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class DriveSysIdMotion implements SwerveMotion {
  private Voltage[] voltages;

  /** The swerve drive this motion controls. */
  private MotionSwerveDrive swerveDrive;

  /**
   * Creates a new ControlTestMotion with the specified control requests.
   *
   * @param swerveDrive The swerve drive to control
   * @param controlRequests Array of control requests (2 per module: drive, steer)
   */
  public DriveSysIdMotion(MotionSwerveDrive swerveDrive, Voltage[] voltages) {
    this.swerveDrive = swerveDrive;
    this.voltages = voltages;
  }

  /**
   * Applies all configured control requests to the swerve modules.
   *
   * <p>This method sends each stored control request to its corresponding motor on every control
   * loop iteration.
   *
   * @param deltaTimeSeconds The time since the last update
   */
  @Override
  public synchronized void update(double deltaTimeSeconds) {
    ChassisSpeeds robotRelativeVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(1, 0, 0), new Rotation2d(swerveDrive.getHeading()));

    SwerveModuleState[] states =
        swerveDrive.getKinematics().toSwerveModuleStates(robotRelativeVelocity);

    double updateFrequencyHz = swerveDrive.getConstants().Timing.ControlLoopFrequency.in(Hertz);
    boolean useTimesync = swerveDrive.getConstants().Timing.TimesyncControlRequests;
    SteerMotorConstants steerConstants = swerveDrive.getConstants().SteerMotor;

    for (int i = 0; i < swerveDrive.getModules().length; i++) {
      SwerveModule module = swerveDrive.getModules()[i];
      SwerveModuleState state = states[i];
      Angle currentAngle = module.getSteerMechanism().getPosition();

      state = SwerveKinematicsUtil.optimize(state, currentAngle);
      state = SwerveKinematicsUtil.decreaseVelocityIfMisaligned(state, currentAngle);

      Angle steerAngle = state.angle.getMeasure();

      module.setControl(
          !voltages[i].equals(Volts.of(0))
              ? new VoltageOut(voltages[i].times(Math.signum(state.speedMetersPerSecond)))
                  .withEnableFOC(false)
              : new CoastOut(),
          new PositionControlRequest(steerAngle.in(Rotations))
              .withMotionProfileType(steerConstants.PositionControlMotionProfile)
              .withOutputType(steerConstants.OutputType)
              .withSlot(steerConstants.PositionSlot)
              .withUpdateFreqHz(updateFrequencyHz)
              .withUseTimesync(useTimesync)
              .toControlRequest());
    }
  }

  @Override
  public SwerveMotion fuseWith(SwerveMotion other) {
    return other;
  }

  /**
   * Logs telemetry data for this control test motion.
   *
   * @param basePath The base path for telemetry logging
   */
  @Override
  public synchronized void logTelemetry(String basePath) {
    if (!basePath.endsWith("/")) {
      basePath += "/";
    }

    DogLog.log(basePath + "Type", "DriveSysId");
    DogLog.log(basePath + "FrontLeftVoltage", voltages[0]);
    DogLog.log(basePath + "FrontRightVoltage", voltages[1]);
    DogLog.log(basePath + "BackLeftVoltage", voltages[2]);
    DogLog.log(basePath + "BackRightVoltage", voltages[3]);
  }
}
