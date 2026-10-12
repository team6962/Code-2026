package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.team6962.lib.phoenix.control.VelocityMotionProfileType;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

public class SwerveMotionProfileConfigurer extends SubsystemBase {
  public enum State {
    AUTO(MetersPerSecond.of(4), VelocityMotionProfileType.Trapezoidal),
    TELEOP(MetersPerSecond.of(5.5), VelocityMotionProfileType.None);

    public final LinearVelocity maxLinearVelocity;
    public final VelocityMotionProfileType motionProfileType;

    private State(LinearVelocity maxLinearVelocity, VelocityMotionProfileType motionProfileType) {
      this.maxLinearVelocity = maxLinearVelocity;
      this.motionProfileType = motionProfileType;
    }
  }

  private State currentState;
  private List<Consumer<State>> listeners = new LinkedList<>();
  private Notifier notifier;

  public SwerveMotionProfileConfigurer() {
    currentState = State.TELEOP;
    notifier =
        new Notifier(
            () -> {
              State state;
              synchronized (this) {
                state = currentState;
              }
              synchronized (listeners) {
                listeners.forEach(
                    func -> {
                      func.accept(state);
                    });
              }
            });
  }

  public void setState(State state) {
    boolean changed = false;

    synchronized (this) {
      if (state != currentState) {
        currentState = state;
        changed = true;
      }
    }

    if (changed) notifier.startSingle(0);
  }

  public State getState() {
    synchronized (this) {
      return currentState;
    }
  }

  public void addListener(Consumer<State> listener) {
    synchronized (listeners) {
      listeners.add(listener);
    }
  }

  @Override
  public void periodic() {
    DogLog.log("SwerveMotionProfileConfigurer/State", getState().toString());

    if (RobotState.isAutonomous()) {
      setState(State.AUTO);
    } else if (RobotState.isTeleop()) {
      setState(State.TELEOP);
    }
  }
}
