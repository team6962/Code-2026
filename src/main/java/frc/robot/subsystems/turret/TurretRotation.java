package frc.robot.subsystems.turret;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.math.MeasureUtil;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This defines the Shooter Roller as a new Subsystem called motor.
* The following StatusSignals represent certain stats of the motor */

public class TurretRotation extends SubsystemBase {
        private TalonFX motor;
        private StatusSignal<AngularVelocity> angVelocitySignal;
        private StatusSignal<Voltage> voltageSignal;
        private StatusSignal<Angle> angleSignal;
        private StatusSignal<AngularAcceleration> angAccelerationSignal;
        private StatusSignal<Current> supplyCurrentSignal;
        private TurretSim simulation;

    /* Assigns StatusSignals to different methods part of the motor.get...() */
    public TurretRotation() {
        motor = new TalonFX(2, new CANBus("drivetrain"));
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1.3;
        motor.getConfigurator().apply(config);
        angVelocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        angleSignal = motor.getPosition();
        angAccelerationSignal = motor.getAcceleration();
        supplyCurrentSignal = motor.getSupplyCurrent();
        if (RobotBase.isSimulation()) {
            simulation = new TurretSim(motor);
        }
    }

    @Override
    public void periodic() {
        if(simulation != null){
            simulation.update();
        }

        /* Motor Statistics are logged as Turret Rotation */
        BaseStatusSignal.refreshAll(angVelocitySignal, voltageSignal, angleSignal, angAccelerationSignal, supplyCurrentSignal);
        DogLog.log("Turret Rotation/Angular Velocity", getAngularVelocity());
        DogLog.log("Turret Rotation/Motor Voltage", getMotorVoltage());
        DogLog.log("Turret Rotation/Motor Position angle", getPosition());
        DogLog.log("Turret Rotation/Angular Acceleration", getAngularVelocity());
        DogLog.log("Turret Rotation/Angular Supply Current", getSupplyCurrent());
    }

    public Current getSupplyCurrent() {
        return supplyCurrentSignal.getValue();
    }
    public AngularAcceleration getAcceleration() {
        return angAccelerationSignal.getValue();
    }
    public AngularVelocity getAngularVelocity() {
        return
            BaseStatusSignal.getLatencyCompensatedValue(
                angVelocitySignal, angAccelerationSignal
            );
    }
    public Voltage getMotorVoltage() {
        return voltageSignal.getValue();
    }
    public Angle getPosition() {
        return MeasureUtil.toAngle(
            BaseStatusSignal.getLatencyCompensatedValue(
                angleSignal, angVelocitySignal
            )
        );
    }

    /* Moves the motor to the target angle */
    public Command moveToleft() {
        return startEnd(()->{
        motor.setControl(new PositionVoltage(1));
        }, ()->{
        motor.setControl(new PositionVoltage(getPosition()));
        });
    }
}
