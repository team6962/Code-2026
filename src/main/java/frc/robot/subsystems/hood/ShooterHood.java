package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.math.MeasureUtil;
import com.ctre.phoenix6.controls.PositionVoltage;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase{
    private TalonFX hoodMotor;

    private StatusSignal<Angle> angle;
    private StatusSignal<AngularVelocity> angVelocity;
    private StatusSignal<AngularAcceleration> angAcceleration;

    private StatusSignal<Voltage> voltage;
    private StatusSignal<Current> current;

    private ShooterHoodSim simulation;
    /**
     * Initializes the motor and status signal
     */
    public ShooterHood() {
        hoodMotor = new TalonFX(39); //change later
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.75;
        hoodMotor.getConfigurator().apply(config);

        angVelocity = hoodMotor.getVelocity();
        voltage = hoodMotor.getMotorVoltage();
        angAcceleration = hoodMotor.getAcceleration();
        angle = hoodMotor.getPosition();
        current = hoodMotor.getSupplyCurrent();

        if (RobotBase.isSimulation()) {
            simulation = new ShooterHoodSim(hoodMotor);
        }
    }
@Override
    public void periodic() {

        if (simulation != null) {
            simulation.update();
        }

        BaseStatusSignal.refreshAll(angVelocity, voltage, angAcceleration, angle, current);
        DogLog.log("Hood Motor/Angle", getPosition());
        DogLog.log("Hood Motor/Angular Velocity", getAngularVelocity());
        DogLog.log("Hood Motor/Angular Acceleration", getAngularAcceleration());
        DogLog.log("Hood Motor/Motor Voltage", getMotorVoltage());
        DogLog.log("Hood Motor/Current", getSupply());
    }
    /**
     * Gets Angular Velocity and Motor Voltage
     */
    public Angle getPosition() {
        return MeasureUtil.toAngle(
            BaseStatusSignal.getLatencyCompensatedValue(
                angle, angVelocity
            )
        );
    }

    public AngularAcceleration getAngularAcceleration() {
        return angAcceleration.getValue();
    }

    public AngularVelocity getAngularVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(
                angVelocity, angAcceleration);
     }

    public Voltage getMotorVoltage() {
        return voltage.getValue();
     }

    public Current getSupply() {
        return current.getValue();
    }

    public Command moveTo(double radians) {
        return startEnd(()->{
        hoodMotor.setControl(new PositionVoltage(radians/(Math.PI*2)));   
        }, ()->{
        hoodMotor.setControl(new PositionVoltage(getPosition()));
        });
     }
}