package frc.robot.subsystems;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This defines the Shooter Roller as a new Subsystem(adds shootermotor, angVelocity, and Voltage)
 */

public class ShooterRoller extends SubsystemBase {
        private TalonFX shootermotor;
        private StatusSignal<AngularVelocity> angVelocity;
        private StatusSignal<Voltage> voltage;
        /**
         * Initializes the motor and status signal
         */
    public ShooterRoller() {
        shootermotor = new TalonFX(1, new CANBus("canivore"));//change this lat
        shootermotor.getConfigurator().apply(
            new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKV(0.12))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false)
            )
        );
        angVelocity = shootermotor.getVelocity();
        voltage = shootermotor.getMotorVoltage();
    }
@Override
    public void periodic() {
        BaseStatusSignal.refreshAll(angVelocity, voltage);
        DogLog.log("Shooter Roller/Angular Velocity", getAngularVelocity());
        DogLog.log("Shooter Roller/Motor Voltage", getMotorVoltage());
    }

/**
 * Gets Angular Velocity and Motor Voltage
 */

    public AngularVelocity getAngularVelocity() {
        return angVelocity.getValue();
    }

    public Voltage getMotorVoltage() {
        return voltage.getValue();
    }

    public Command driveSlow() {
        return startEnd(()->{
        shootermotor.setControl(new VelocityVoltage(30));
        }, ()->{
        shootermotor.setControl(new CoastOut());
        });
    }
  
    public Command coastOut() {
        return run(()->{
            shootermotor.setControl(new CoastOut());
        });
    }
    public Command driveNeg() {
        return startEnd(()->{
        shootermotor.setControl(new VelocityVoltage(-30));
        }, ()->{
        shootermotor.setControl(new CoastOut());        
        });
    }
}