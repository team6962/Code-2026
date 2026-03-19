package com.team6962.lib.swerve.pathplanner;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Newtons;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.swerve.motion.VelocityMotion;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;

public class PathPlanner {
  private CommandSwerveDrive drivetrain;
  private ChassisSpeeds lastSpeeds;
  private boolean trackingTranslation = false;
  private boolean trackingRotation = false;
  private Map<String, PathPlannerPath> mirroredPaths = new HashMap<>();

  public PathPlanner(CommandSwerveDrive drivetrain) {
    this.drivetrain = drivetrain;

    AutoBuilder.configure(
        drivetrain::getPosition2d,
        drivetrain.getLocalization()::resetPosition,
        drivetrain::getVelocity,
        (speeds, ff) -> {
          ChassisSpeeds appliedSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(speeds, new Rotation2d(drivetrain.getYaw()));

          if (lastSpeeds != null) {
            appliedSpeeds.vxMetersPerSecond +=
                (appliedSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond)
                    / 0.02
                    * drivetrain.getConstants().Driving.AutoLinearAccelerationScalar;
            appliedSpeeds.vyMetersPerSecond +=
                (appliedSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond)
                    / 0.02
                    * drivetrain.getConstants().Driving.AutoLinearAccelerationScalar;
            appliedSpeeds.omegaRadiansPerSecond +=
                (appliedSpeeds.omegaRadiansPerSecond - lastSpeeds.omegaRadiansPerSecond)
                    / 0.02
                    * drivetrain.getConstants().Driving.AutoAngularAccelerationScalar;
          }

          if (!trackingTranslation) {
            appliedSpeeds.vxMetersPerSecond = 0;
            appliedSpeeds.vyMetersPerSecond = 0;
          }

          if (!trackingRotation) {
            appliedSpeeds.omegaRadiansPerSecond = 0;
          }

          Force[] forces = new Force[] {Newtons.of(0), Newtons.of(0), Newtons.of(0), Newtons.of(0)};

          if (trackingRotation && trackingTranslation) {
            SwerveModulePosition[] modulePositions = drivetrain.getModulePositions();

            for (int i = 0; i < 4; i++) {
              Rotation2d steerAngle = modulePositions[i].angle;
              Vector<N2> steerDirectionVector =
                  VecBuilder.fill(steerAngle.getCos(), steerAngle.getSin());
              Vector<N2> forceVector =
                  VecBuilder.fill(
                      ff.robotRelativeForcesXNewtons()[i], ff.robotRelativeForcesYNewtons()[i]);
              double forceNewtons = steerDirectionVector.dot(forceVector);
              forces[i] = Newtons.of(forceNewtons);
            }
          }

          drivetrain.applyMotion(new VelocityMotion(appliedSpeeds, forces, drivetrain));
        },
        new PPHolonomicDriveController(
            new PIDConstants(
                drivetrain.getConstants().Driving.TranslationFeedbackKP,
                drivetrain.getConstants().Driving.TranslationFeedbackKI,
                drivetrain.getConstants().Driving.TranslationFeedbackKD),
            new PIDConstants(
                drivetrain.getConstants().Driving.AngleFeedbackKP,
                drivetrain.getConstants().Driving.AngleFeedbackKI,
                drivetrain.getConstants().Driving.AngleFeedbackKD)),
        new RobotConfig(
            drivetrain.getConstants().Structure.RobotMass,
            drivetrain.getConstants().Structure.RobotMomentOfInertia,
            new ModuleConfig(
                drivetrain.getConstants().Structure.WheelRadius,
                drivetrain.getConstants().DriveMotor.MaxVelocity,
                drivetrain.getConstants().Structure.WheelCOF,
                drivetrain.getConstants().DriveMotor.SimulatedMotor,
                Amps.of(
                    drivetrain.getConstants()
                        .DriveMotor
                        .DeviceConfiguration
                        .CurrentLimits
                        .SupplyCurrentLimit),
                1),
            drivetrain.getConstants().Structure.getModuleTranslations()),
        () -> false);

    PathPlannerLogging.setLogActivePathCallback(
        poses -> {
          drivetrain.getFieldLogger().getField().getObject("Active Path").setPoses(poses);
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> {
          drivetrain.getFieldLogger().getField().getObject("Target Pose").setPose(pose);
        });
  }

  private Command followPath(
      String pathName, PathPlannerPath path, boolean trackRotation, boolean trackTranslation) {
    String commandName;

    if (trackRotation && trackTranslation) {
      commandName = "Follow Path: " + pathName;
    } else if (trackRotation) {
      commandName = "Follow Path Rotation: " + pathName;
    } else if (trackTranslation) {
      commandName = "Follow Path Translation: " + pathName;
    } else {
      commandName = "Follow Path (No Tracking): " + pathName;
    }

    if (path == null) {
      return Commands.print("Attempting to follow null path: " + pathName).withName(commandName);
    }

    Command pathPlannerCommand = AutoBuilder.followPath(path);
    Command wrapperCommand =
        new Command() {
          @Override
          public void initialize() {
            trackingRotation = trackRotation;
            trackingTranslation = trackTranslation;
            lastSpeeds = null;
            pathPlannerCommand.initialize();
          }

          @Override
          public void execute() {
            pathPlannerCommand.execute();
          }

          @Override
          public void end(boolean interrupted) {
            pathPlannerCommand.end(interrupted);
            trackingRotation = false;
            trackingTranslation = false;
            lastSpeeds = null;
          }

          @Override
          public boolean isFinished() {
            return pathPlannerCommand.isFinished();
          }
        };

    wrapperCommand.setName(commandName);

    if (trackRotation) {
      wrapperCommand.addRequirements(drivetrain.useRotation());
    }

    if (trackTranslation) {
      wrapperCommand.addRequirements(drivetrain.useTranslation());
    }

    return wrapperCommand;
  }

  public PathPlannerPath loadChoreoPath(String pathName, boolean mirrorPath) {
    try {
      PathPlannerPath unmirroredPath = PathPlannerPath.fromChoreoTrajectory(pathName);

      if (mirrorPath) {
        return mirroredPaths.computeIfAbsent(pathName, key -> unmirroredPath.mirrorPath());
      } else {
        return unmirroredPath;
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + pathName, true);
      e.printStackTrace();
      return null;
    }
  }

  public PathPlannerPath loadChoreoPath(String pathName, int splitIndex, boolean mirrorPath) {
    try {
      PathPlannerPath unmirroredPath = PathPlannerPath.fromChoreoTrajectory(pathName, splitIndex);

      if (mirrorPath) {
        return mirroredPaths.computeIfAbsent(
            pathName + "." + splitIndex, key -> unmirroredPath.mirrorPath());
      } else {
        return unmirroredPath;
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + pathName + "." + splitIndex, true);
      e.printStackTrace();
      return null;
    }
  }

  public PathPlannerPath loadChoreoPath(String pathName) {
    loadChoreoPath(pathName, true);
    return loadChoreoPath(pathName, false);
  }

  public PathPlannerPath loadChoreoPath(String pathName, int splitIndex) {
    loadChoreoPath(pathName, splitIndex, true);
    return loadChoreoPath(pathName, splitIndex, false);
  }

  private Command followPath(
      String pathName, boolean trackRotation, boolean trackTranslation, boolean mirrorPath) {
    if (pathName.contains(".")) {
      // Use the split index if it's included in the path name
      String[] parts = pathName.split("\\.");

      if (parts.length == 2) {
        try {
          int splitIndex = Integer.parseInt(parts[1]);
          return followPath(parts[0], splitIndex, trackRotation, trackTranslation, mirrorPath);
        } catch (NumberFormatException e) {
          DriverStation.reportError("Invalid split index in path name: " + pathName, true);
        }
      }
    }

    return followPath(
        pathName, loadChoreoPath(pathName, mirrorPath), trackRotation, trackTranslation);
  }

  private Command followPath(
      String pathName,
      int splitIndex,
      boolean trackRotation,
      boolean trackTranslation,
      boolean mirrorPath) {
    return followPath(
        pathName + "." + splitIndex,
        loadChoreoPath(pathName, splitIndex, mirrorPath),
        trackRotation,
        trackTranslation);
  }

  public Command followPath(String pathName, boolean mirrorPath) {
    return followPath(pathName, true, true, mirrorPath);
  }

  public Command followPathRotation(String pathName, boolean mirrorPath) {
    return followPath(pathName, true, false, mirrorPath);
  }

  public Command followPathTranslation(String pathName, boolean mirrorPath) {
    return followPath(pathName, false, true, mirrorPath);
  }

  public Command followPath(String pathName, int splitIndex, boolean mirrorPath) {
    return followPath(pathName, splitIndex, true, true, mirrorPath);
  }

  public Command followPathRotation(String pathName, int splitIndex, boolean mirrorPath) {
    return followPath(pathName, splitIndex, true, false, mirrorPath);
  }

  public Command followPathTranslation(String pathName, int splitIndex, boolean mirrorPath) {
    return followPath(pathName, splitIndex, false, true, mirrorPath);
  }

  public Command followPath(String pathName) {
    return followPath(pathName, false);
  }

  public Command followPathRotation(String pathName) {
    return followPathRotation(pathName, false);
  }

  public Command followPathTranslation(String pathName) {
    return followPathTranslation(pathName, false);
  }

  public Command followPath(String pathName, int splitIndex) {
    return followPath(pathName, splitIndex, false);
  }

  public Command followPathRotation(String pathName, int splitIndex) {
    return followPathRotation(pathName, splitIndex, false);
  }

  public Command followPathTranslation(String pathName, int splitIndex) {
    return followPathTranslation(pathName, splitIndex, false);
  }
}
