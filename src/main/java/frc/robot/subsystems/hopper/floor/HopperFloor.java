package frc.robot.subsystems.hopper.floor;

import edu.wpi.first.wpilibj2.command.Command;

public interface HopperFloor {
  public Command feed();

  public Command dump();
}
