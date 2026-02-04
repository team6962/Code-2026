package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class IntakeRollersConstants {
  public static final double gearRatio = 5.0;
  public static final double momentOfInertia = 0.001;
  public static final int deviceId = 41;
  public static final Current statorCurrentLimit = Amps.of(120);
  public static final Current supplyCurrentLimit = Amps.of(60);
}
