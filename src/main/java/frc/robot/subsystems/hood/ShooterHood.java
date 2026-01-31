package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.MeasureUtil;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.networktables.DoubleSubscriber;

public class ShooterHood extends SubsystemBase{
    private TalonFX hoodMotor;

    private StatusSignal<Angle> angle;
    private StatusSignal<AngularVelocity> angVelocity;
    private StatusSignal<AngularAcceleration> angAcceleration;

    private StatusSignal<Voltage> voltage;
    private StatusSignal<Current> current;

    private ShooterHoodSim simulation;
    private final DoubleSubscriber angleInput;
    /**
     * Initializes the motor and status signal
     */
    public ShooterHood() {
        hoodMotor = new TalonFX(20); //change later
        // TalonFXConfiguration config = new TalonFXConfiguration();
        // config.Slot0.kP = 0.75;
        // config.Slot0.kD = 0.1;
        // config.Slot0.kS = 0.15;
        // config.Slot0.kV = 2.571;
        // config.Slot0.kA = 0.030;

        // config.MotionMagic.MotionMagicCruiseVelocity = 10;
        // config.MotionMagic.MotionMagicAcceleration = 5;

        // config.Feedback.RotorToSensorRatio = 150.0 /7.0;

        hoodMotor.getConfigurator().apply(ShooterHoodConstants.MOTOR_CONFIGURATION);

        angVelocity = hoodMotor.getVelocity();
        voltage = hoodMotor.getMotorVoltage();
        angAcceleration = hoodMotor.getAcceleration();
        angle = hoodMotor.getPosition();
        current = hoodMotor.getSupplyCurrent();

        angleInput = DogLog.tunable("Hood Motor/Input Angle", 0.0, newAngle -> {
            moveTo(newAngle).schedule();
        });

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
        DogLog.log("Hood Motor/Angle", getPosition().in(Degrees));
        DogLog.log("Hood Motor/Angular Velocity", getAngularVelocity());
        DogLog.log("Hood Motor/Angular Acceleration", getAngularAcceleration());
        DogLog.log("Hood Motor/Motor Voltage", getMotorVoltage());
        DogLog.log("Hood Motor/Current", getSupply());

        LoggingUtil.log("Hood Motor/Control Request", hoodMotor.getAppliedControl());
    }

    public Angle clampPositionToSafeRange(Angle input) {
        if (input.gt(ShooterHoodConstants.MAX_ANGLE)) {
            return ShooterHoodConstants.MAX_ANGLE;
        }
        else if (input.lt(ShooterHoodConstants.MIN_ANGLE)) {
            return ShooterHoodConstants.MIN_ANGLE;
        }
        return input;
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
        Angle clampedAngle = clampPositionToSafeRange(Radians.of(radians));

        DogLog.log("Hood Motor/Target Angle", clampedAngle.in(Degrees));

        return startEnd(()->{
        hoodMotor.setControl(new MotionMagicVoltage(clampedAngle));   
        }, ()->{
            System.out.println(getPosition());
        hoodMotor.setControl(new MotionMagicVoltage(getPosition()));
        });
     }
}