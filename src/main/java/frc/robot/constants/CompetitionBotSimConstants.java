package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.team6962.lib.swerve.config.DrivetrainConstants;

/**
 * Simulation-only tuning for the competition robot.
 *
 * <p>This keeps simulation realism improvements isolated from the real robot constants so we can
 * make autos and driver practice behave better in sim without changing match behavior on hardware.
 */
public class CompetitionBotSimConstants extends CompetitionBotConstants {
  @Override
  public DrivetrainConstants getDrivetrainConstants() {
    DrivetrainConstants constants = super.getDrivetrainConstants();

    return constants
        .withStructure(constants.Structure.clone().estimateMomentOfInertia())
        .withDriving(
            constants.Driving.clone()
                .withAutoLinearVelocity(MetersPerSecond.of(3.2))
                .withAutoLinearAcceleration(MetersPerSecondPerSecond.of(2.6))
                .withAutoAngularVelocity(RotationsPerSecond.of(0.85))
                .withAutoAngularAcceleration(RotationsPerSecondPerSecond.of(0.4))
                .withTranslationFeedbackKP(8.0)
                .withTranslationFeedbackKD(0.45)
                .withAngleFeedbackKP(16.0)
                .withAngleFeedbackKD(0.9))
        .withSimulation(
            constants.Simulation.clone()
                .withSimulationSubTicksPerPeriod(4)
                .withEnableRampCollider(true));
  }
}
