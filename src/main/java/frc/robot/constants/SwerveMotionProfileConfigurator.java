package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.team6962.lib.phoenix.control.VelocityMotionProfileType;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

public class SwerveMotionProfileConfigurator extends SubsystemBase {
  public enum State {
    AUTO(MetersPerSecond.of(3.5), VelocityMotionProfileType.Trapezoidal),
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

  public SwerveMotionProfileConfigurator() {
    currentState = State.TELEOP;
  }

  public void setState(State state) {
    if (state != currentState) {
      currentState = state;
      for (Consumer<State> listener : listeners) {
        listener.accept(state);
      }
    }
  }

  public State getState() {
    return currentState;
  }

  public void addListener(Consumer<State> listener) {
    listeners.add(listener);
    listener.accept(currentState);
  }

  @Override
  public void periodic() {
    DogLog.log("SwerveMotionProfileConfigurator/State", getState().toString());

    if (RobotState.isAutonomous()) {
      setState(State.AUTO);
    } else if (RobotState.isTeleop()) {
      setState(State.TELEOP);
    }
  }
}
