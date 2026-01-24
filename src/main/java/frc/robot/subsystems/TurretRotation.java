package frc.robot.subsystems;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MusicTone;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.Orchestra;

/**
 * This defines the Shooter Roller as a new Subsystem(adds shootermotor, angVelocity, and Voltage)
 */

public class TurretRotation extends SubsystemBase {
        private TalonFX motor;
        private TalonFX motor2;
        private TalonFX motor3;
        private TalonFX motor4;
        private TalonFX motor5;
        private TalonFX motor6;
        private TalonFX motor7;
        private TalonFX motor8;
        private StatusSignal<AngularVelocity> angVelocitySignal;
        private StatusSignal<Voltage> voltageSignal;
        private StatusSignal<Angle> angleSignal;
        private StatusSignal<AngularAcceleration> angAccelerationSignal;
        private StatusSignal<Current> supplyCurrentSignal;
        private TurretSim simulation;
        private Timer timer;
        private final Orchestra orchestra = new Orchestra();

    /**
    * Initializes the motor and status signal
    */
    public TurretRotation() {
        motor = new TalonFX(0, new CANBus("drivetrain"));
        motor2 = new TalonFX(1, new CANBus("drivetrain"));
        motor3 = new TalonFX(2, new CANBus("drivetrain"));
        motor4 = new TalonFX(3, new CANBus("drivetrain"));
        motor5 = new TalonFX(4, new CANBus("drivetrain"));
        motor6 = new TalonFX(5, new CANBus("drivetrain"));
        motor7 = new TalonFX(6, new CANBus("drivetrain"));
        motor8 = new TalonFX(7, new CANBus("drivetrain"));
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1.2;
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
        BaseStatusSignal.refreshAll(angVelocitySignal, voltageSignal, angleSignal, angAccelerationSignal, supplyCurrentSignal);
        DogLog.log("Turret Rotation/Angular Velocity", getAngularVelocity());
        DogLog.log("Turret Rotation/Motor Voltage", getMotorVoltage());
        DogLog.log("Turret Rotation/Motor Position angle", getPosition());
        DogLog.log("Turret Rotation/Angular Acceleration", getAngularVelocity());
        DogLog.log("Turret Rotation/Angular Supply Current", getSupplyCurrent());
    }

    public Command playSong() {
    return run(() -> {})
        .beforeStarting(() -> {
          orchestra.addInstrument(motor);
          orchestra.addInstrument(motor2);
          orchestra.addInstrument(motor3);
          orchestra.addInstrument(motor4);
          orchestra.addInstrument(motor5);
          orchestra.addInstrument(motor6);
          orchestra.addInstrument(motor7);
          orchestra.addInstrument(motor8);
          var status = orchestra.loadMusic("song.chrp"); // Put the chrp fie in the deploy folder
          if (!status.isOK()) {
            DogLog.log("CHRP FAILED TO LOAD", true);
          }
          orchestra.play();
        })
        .finallyDo(() -> {
          orchestra.stop();
          motor.setControl(new CoastOut());
        });
    }

    public Command playTone(float tone) {
        return run(() -> motor.setControl(new MusicTone(tone)))
            .beforeStarting(() -> {
            timer.reset();
            timer.start();
            })
            .finallyDo(() -> {
            timer.stop();
            motor.setControl(new CoastOut());
            });
    }

    /**
     * Gets Angular Velocity and Motor Voltage
     */
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

    public Command moveToleft() {
        return startEnd(()->{
        motor.setControl(new PositionVoltage(1));
        }, ()->{
        motor.setControl(new PositionVoltage(0));
        });
    }
}