package frc.robot.subsystems;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.phoenix.StatusUtil;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem class is reponsible for controlling the turret's rotation.
 */
public class TurretRotation extends SubsystemBase {
    /**
     * The TalonFX motor controller used to control the turret rotation motor.
     */
    private TalonFX motor;

    /**
     * Status signal for the position of the turret motor.
     */
    private StatusSignal<Angle> positionSignal;

    /**
     * Status signal for the angular velocity of the turret motor.
     */
    private StatusSignal<AngularVelocity> velocitySignal;

    /**
     * Status signal for the angular acceleration of the turret motor.
     */
    private StatusSignal<AngularAcceleration> accelerationSignal;

    /**
     * Status signal for the applied voltage to the turret motor.
     */
    private StatusSignal<Voltage> voltageSignal;

    /**
     * Status signal for the supply current drawn by the turret motor.
     */
    private StatusSignal<Current> supplyCurrentSignal;

    /**
     * Status signal for the stator current of the turret motor.
     */
    private StatusSignal<Current> statorCurrentSignal;

    /**
     * Simulation object for the turret rotation subsystem, if in simulation
     * mode. When not in simulation mode, this will be null.
     */
    private TurretSim simulation;

    /**
     * Creates a new TurretRotation subsystem, which can be used to control the
     * turret's rotation.
     */
    public TurretRotation() {
        // Initialize motor controller object
        motor = new TalonFX(2, new CANBus("drivetrain"));

        // Configure motor controller
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 6;
        config.Feedback.RotorToSensorRatio = 150.0 / 7.0;

        StatusUtil.check(motor.getConfigurator().apply(config));

        // Get status signals
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        accelerationSignal = motor.getAcceleration();
        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();

        // Set up simulation if in simulation mode
        if (RobotBase.isSimulation()) {
            simulation = new TurretSim(motor);
        }
    }
    
    @Override
    public void periodic() {
        // Update simulation if required
        if (simulation != null) {
            simulation.update();
        }

        // If disabled, set target to current position
        if (RobotState.isDisabled()) {
            setControl(new PositionVoltage(getPosition()));
        }

        // Refresh status signals, pulling the latest data from the motor
        // controller
        BaseStatusSignal.refreshAll(
            positionSignal, velocitySignal, accelerationSignal,
            voltageSignal, supplyCurrentSignal, statorCurrentSignal
        );

        // Log relevant data to DogLog
        DogLog.log("TurretRotation/Position", getPosition());
        DogLog.log("TurretRotation/Velocity", getVelocity());
        DogLog.log("TurretRotation/Acceleration", getAcceleration());
        DogLog.log("TurretRotation/AppliedVoltage", getAppliedVoltage());
        DogLog.log("TurretRotation/SupplyCurrent", getSupplyCurrent());
        DogLog.log("TurretRotation/StatorCurrent", getStatorCurrent());
    }

    /**
     * Gets the current position of the turret. An angle of 0 degrees is facing
     * towards the intake side of the robot, and positive angles rotate
     * counterclockwise when looking down on the robot from above.
     * 
     * @return The current position of the turret, as an {@link Angle}.
     */
    public Angle getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(
            positionSignal, velocitySignal
        );
    }

    /**
     * Gets the current angular velocity of the turret.
     * 
     * @return The current angular velocity of the turret, as an {@link AngularVelocity}.
     */
    public AngularVelocity getVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(
            velocitySignal, accelerationSignal
        );
    }

    /**
     * Gets the current angular acceleration of the turret.
     * 
     * @return The current angular acceleration of the turret, as an {@link AngularAcceleration}.
     */
    public AngularAcceleration getAcceleration() {
        return accelerationSignal.getValue();
    }

    /**
     * Gets the applied voltage to the turret motor.
     * 
     * @return The applied voltage, as a {@link Voltage}.
     */
    public Voltage getAppliedVoltage() {
        return voltageSignal.getValue();
    }

    /**
     * Gets the supply current drawn by the turret motor.
     * 
     * @return The supply current, as a {@link Current}.
     */
    public Current getSupplyCurrent() {
        return supplyCurrentSignal.getValue();
    }

    /**
     * Gets the stator current of the turret motor.
     * 
     * @return The stator current, as a {@link Current}.
     */
    public Current getStatorCurrent() {
        return statorCurrentSignal.getValue();
    }

    /**
     * Moves the turret to the specified target angle.
     * 
     * @param target The target angle to move the turret to. An angle of 0
     *               degrees is facing towards the intake side of the robot,
     *               and positive angles rotate counterclockwise when looking
     *               down on the robot from above.
     * @return A command that moves the turret to the specified angle.
     */
    public Command moveTo(Angle target) {
        return startEnd(() -> {
            setControl(new PositionVoltage(target));
        }, () -> {
            setControl(new PositionVoltage(getPosition()));
        });
    }

    /**
     * Applies a control request and logs it.
     * 
     * @param controlRequest The control request to apply.
     */
    private void setControl(ControlRequest controlRequest) {
        LoggingUtil.log("TurretRotation/ControlRequest", controlRequest);

        StatusUtil.check(motor.setControl(controlRequest));
    }
}