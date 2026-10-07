package frc.robot.controls;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerManager extends SubsystemBase {
    public static enum State {
        TELEOP
        (35, 60, 60, 20, 40, 40, 30, 40),
        SHOOTING(20, 60, 40, 20, 40, 40, 10, 20),
        TURBO(70, 60, 60, 40, 40, 40, 60, 60),
        AUTO(70, 60, 40, 20, 40, 40, 30, 60);

        public final double driveCurrentLimit;
        public final double shooterRollerCurrentLimit;
        public final double shooterTurretCurrentLimit;
        public final double shooterHoodCurrentLimit;
        public final double hopperCurrentLimit;
        public final double queueCurrentLimit;
        public final double intakeExtensionCurrentLimit;
        public final double intakeRollerCurrentLimit;

        private State(double dr, double sh, double tr, double hd, double hp, double q, double ext, double in) {
            this.driveCurrentLimit = dr;
            this.shooterRollerCurrentLimit = sh;
            this.shooterTurretCurrentLimit = tr;
            this.shooterHoodCurrentLimit = hd;
            this.hopperCurrentLimit = hp;
            this.queueCurrentLimit = q;
            this.intakeExtensionCurrentLimit = ext;
            this.intakeRollerCurrentLimit = in;
        }
    }

    private State currentState;
    private List<Consumer<State>> listeners = new LinkedList<>();
    private Notifier notifier;

    public PowerManager() {
        currentState = State.TELEOP;
        notifier = new Notifier(() -> {
            State state;
            synchronized (this) {
                state = currentState;
            }
            synchronized (listeners) {
                listeners.forEach(func -> {
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
        DogLog.log("PowerManager/State", getState().toString());
        DogLog.log("PowerManager/BatteryVoltage", RobotController.getBatteryVoltage());

        if (RobotState.isAutonomous()) {
            setState(State.TELEOP);
        } else if (RobotState.isDisabled() && RobotState.isTeleop()) {
            setState(State.TELEOP);
        }
    }
}
