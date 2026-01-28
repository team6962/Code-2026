package frc.robot.subsystems.abstraction;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public interface ITurret extends Subsystem {
  public Angle getPosition();

  public AngularVelocity getVelocity();

  public Current getSupplyCurrent();

  public Command moveTo(Supplier<Angle> position);
}
