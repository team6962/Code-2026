package frc.robot.controls;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import java.util.function.DoubleSupplier;

/**
 * Subsystem that manages controller rumble. This subsystem allows multiple commands to request
 * rumble simultaneously, and applies the maximum requested rumble each cycle.
 */
public class ControllerRumble extends SubsystemBase {
  /** The controller that will be rumbled. */
  private GenericHID controller;

  /** The current applied output for the left rumble motor. */
  private double leftAppliedOutput;

  /** The current applied output for the right rumble motor. */
  private double rightAppliedOutput;

  /**
   * Creates a new ControllerRumble subsystem for the given controller.
   *
   * @param controller The controller to manage rumble for.
   */
  public ControllerRumble(GenericHID controller) {
    this.controller = controller;
  }

  /**
   * Creates a new ControllerRumble subsystem for the given controller.
   *
   * @param controller The controller to manage rumble for.
   */
  public ControllerRumble(CommandGenericHID controller) {
    this(controller.getHID());
  }

  /**
   * Returns a command that will rumble the controller with the given outputs. If multiple commands
   * are scheduled that request rumble, the controller will apply the maximum requested output from
   * all commands. The rumble will be applied momentarily each cycle, so commands should be
   * continuously scheduled to maintain rumble. The rumble will be applied even when the robot is
   * disabled, to allow for feedback during the disabled period.
   *
   * @param leftRequestedOutput The requested output for the left rumble motor, ranging from 0.0 to
   *     1.0.
   * @param rightRequestedOutput The requested output for the right rumble motor, ranging from 0.0
   *     to 1.0.
   * @return A command that will apply the requested rumble outputs to the controller.
   */
  public Command rumble(double leftRequestedOutput, double rightRequestedOutput) {
    return Commands.run(() -> rumbleMomentary(leftRequestedOutput, rightRequestedOutput))
        .ignoringDisable(true);
  }

  /**
   * Returns a command that will rumble the controller with the given output for both rumble motors.
   * If multiple commands are scheduled that request rumble, the controller will apply the maximum
   * requested output from all commands. The rumble will be applied momentarily each cycle, so
   * commands should be continuously scheduled to maintain rumble. The rumble will be applied even
   * when the robot is disabled, to allow for feedback during the disabled period.
   *
   * @param requestedOutput The requested output for both rumble motors, ranging from 0.0 to 1.0.
   * @return A command that will apply the requested rumble output to both rumble motors on the
   *     controller.
   */
  public Command rumble(DoubleSupplier leftRequestedOutput, DoubleSupplier rightRequestedOutput) {
    return Commands.run(
            () ->
                rumbleMomentary(
                    leftRequestedOutput.getAsDouble(), rightRequestedOutput.getAsDouble()))
        .ignoringDisable(true);
  }

  /**
   * Returns a command that will rumble the controller with the given output for both rumble motors.
   * If multiple commands are scheduled that request rumble, the controller will apply the maximum
   * requested output from all commands. The rumble will be applied momentarily each cycle, so
   * commands should be continuously scheduled to maintain rumble. The rumble will be applied even
   * when the robot is disabled, to allow for feedback during the disabled period.
   *
   * @param requestedOutput The requested output for both rumble motors, ranging from 0.0 to 1.0.
   * @return A command that will apply the requested rumble output to both rumble motors on the
   *     controller.
   */
  public Command rumble(double requestedOutput) {
    return rumble(requestedOutput, requestedOutput);
  }

  /**
   * Returns a command that will rumble the controller with the given output for both rumble motors.
   * If multiple commands are scheduled that request rumble, the controller will apply the maximum
   * requested output from all commands. The rumble will be applied momentarily each cycle, so
   * commands should be continuously scheduled to maintain rumble. The rumble will be applied even
   * when the robot is disabled, to allow for feedback during the disabled period.
   *
   * @param requestedOutput The requested output for both rumble motors, ranging from 0.0 to 1.0.
   * @return A command that will apply the requested rumble output to both rumble motors on the
   *     controller.
   */
  public Command rumble(DoubleSupplier requestedOutput) {
    return rumble(requestedOutput, requestedOutput);
  }

  /**
   * Returns a command that will rumble the controller with the maximum output for both rumble
   * motors. If multiple commands are scheduled that request rumble, the controller will apply the
   * maximum requested output from all commands. The rumble will be applied momentarily each cycle,
   * so commands should be continuously scheduled to maintain rumble. The rumble will be applied
   * even when the robot is disabled, to allow for feedback during the disabled period.
   *
   * @return A command that will apply the maximum rumble output to both rumble motors on the
   *     controller.
   */
  public Command rumble() {
    return rumble(1);
  }

  /**
   * Applies the given rumble outputs momentarily. This method is called by the rumble commands each
   * cycle to apply the requested rumble outputs. Each periodic cycle, the applied outputs are reset
   * to 0, so commands must continuously call this method to maintain rumble. If multiple commands
   * call this method in the same cycle, the maximum requested output will be applied.
   *
   * @param leftOutput The requested output for the left rumble motor, ranging from 0.0 to 1.0.
   * @param rightOutput The requested output for the right rumble motor, ranging from 0.0 to 1.0.
   */
  public void rumbleMomentary(double leftOutput, double rightOutput) {
    leftAppliedOutput = Math.max(leftAppliedOutput, leftOutput);
    rightAppliedOutput = Math.max(rightAppliedOutput, rightOutput);
  }

  @Override
  public void periodic() {
    DogLog.log(
        "Controllers/" + ControllerLogging.getControllerName(controller.getPort()) + "/LeftRumble",
        leftAppliedOutput);
    DogLog.log(
        "Controllers/" + ControllerLogging.getControllerName(controller.getPort()) + "/RightRumble",
        rightAppliedOutput);
    controller.setRumble(XboxController.RumbleType.kLeftRumble, leftAppliedOutput);
    controller.setRumble(XboxController.RumbleType.kRightRumble, rightAppliedOutput);
    leftAppliedOutput = 0;
    rightAppliedOutput = 0;
  }
}
