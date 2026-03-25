package frc.robot.subsystems.hopper.floor;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public interface HopperFloor {
  public Command feedDump(Voltage targetVoltage);

  public Command feed();

  public Command reverse();

  public Command slowReverse();

  public Command dump();
}
