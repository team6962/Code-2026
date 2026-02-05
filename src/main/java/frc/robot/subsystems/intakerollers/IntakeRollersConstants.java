package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class IntakeRollersConstants {
    public static final double GEAR_RATIO = 5.0;
    public static final double MOMENT_OF_INERTIA = 0.00074271944;
    public static final int DEVICE_ID = 41;
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
}
