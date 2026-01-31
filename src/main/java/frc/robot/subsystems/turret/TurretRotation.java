package frc.robot.subsystems.turret;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.MeasureUtil;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
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
        private final DoubleSubscriber angleInput;

        private final DoubleSubscriber kPInput;
        private final DoubleSubscriber kDInput;
        private final DoubleSubscriber kSInput;
        private final DoubleSubscriber kVInput;
        private final DoubleSubscriber kAInput;

        private final DoubleSubscriber cruiseVelocityInput;
        private final DoubleSubscriber accelerationInput;

        private TalonFXConfiguration config;

/**
* Assignes the Status Signal variables to the different methods part of the motor.get...()
*/
    public TurretRotation() {
        motor = new TalonFX(2, new CANBus("drivetrain"));
        config = new TalonFXConfiguration();
        config.Slot0.kP = 0.5;
        config.Slot0.kD = 0.1;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0.681;
        config.Slot0.kA = 0.0036;

        config.MotionMagic.MotionMagicCruiseVelocity = 1;
        config.MotionMagic.MotionMagicAcceleration = 1;
       
        config.Feedback.RotorToSensorRatio = 34.5;

        motor.getConfigurator().apply(config);
        angVelocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        angleSignal = motor.getPosition();
        angAccelerationSignal = motor.getAcceleration();
        supplyCurrentSignal = motor.getSupplyCurrent();

        // Tunable angle input, PID values, Motion Magic stuff
        angleInput = DogLog.tunable("Turret Rotation/Input Angle", 0.0, newAngle -> {
            moveTo(Radians.of(newAngle)).schedule();
        });
        
        kPInput = DogLog.tunable("Turret Rotation/PID/kP", config.Slot0.kP, newKP -> {
            updatePIDConfig();
        });
        
        kDInput = DogLog.tunable("Turret Rotation/PID/kD", config.Slot0.kD, newKD -> {
            updatePIDConfig();
        });
        
        kSInput = DogLog.tunable("Turret Rotation/Feedforward/kS", config.Slot0.kS, newKS -> {
            updatePIDConfig();
        });
        
        kVInput = DogLog.tunable("Turret Rotation/Feedforward/kV", config.Slot0.kV, newKV -> {
            updatePIDConfig();
        });
        
        kAInput = DogLog.tunable("Turret Rotation/Feedforward/kA", config.Slot0.kA, newKA -> {
            updatePIDConfig();
        });
        
        cruiseVelocityInput = DogLog.tunable("Turret Rotation/Motion Magic/Cruise Velocity", 
            config.MotionMagic.MotionMagicCruiseVelocity, newVel -> {
            updatePIDConfig();
        });
        
        accelerationInput = DogLog.tunable("Turret Rotation/Motion Magic/Acceleration", 
            config.MotionMagic.MotionMagicAcceleration, newAccel -> {
            updatePIDConfig();
        });

        if (RobotBase.isSimulation()) {
            simulation = new TurretSim(motor);
        }
    }
    
    /**
    * Updates motor configuration with current tunable values
    */
    private void updatePIDConfig() {
        config.Slot0.kP = kPInput.get();
        config.Slot0.kD = kDInput.get();
        config.Slot0.kS = kSInput.get();
        config.Slot0.kV = kVInput.get();
        config.Slot0.kA = kAInput.get();
        
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocityInput.get();
        config.MotionMagic.MotionMagicAcceleration = accelerationInput.get();
        
        motor.getConfigurator().apply(config);
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

        LoggingUtil.log("Turret Rotation/Control Request", motor.getAppliedControl());
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
        return startEnd(
            () -> motor.setControl(new MotionMagicVoltage(1)),
            () -> motor.setControl(new MotionMagicVoltage(0))
        );
    }

    public Command moveTo(Angle targetAngle) {
        return startEnd(
            () -> {
                motor.setControl(new MotionMagicVoltage(targetAngle));
            },
            () -> {
                motor.setControl(new MotionMagicVoltage(getPosition()));
            }
        );
    }

    public Command moveTo(double targetAngle) {
        return startEnd(
            () -> {
                motor.setControl(new MotionMagicVoltage(targetAngle));
            },
            () -> {
                motor.setControl(new MotionMagicVoltage(getPosition()));
            }
        );
    }
}