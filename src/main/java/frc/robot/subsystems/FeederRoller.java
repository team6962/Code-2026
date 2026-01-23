package frc.robot.subsystems;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederRoller extends SubsystemBase{
    private TalonFX feedermotor;
    private StatusSignal<AngularVelocity> angVelocity;
    private StatusSignal<Voltage> voltage;
    /**
     * Initializes the motor and status signal
     */
    public FeederRoller() {
        feedermotor = new TalonFX(1, new CANBus("canivore")); //change later
        feedermotor.getConfigurator().apply(
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
        angVelocity = feedermotor.getVelocity();
        voltage = feedermotor.getMotorVoltage();
    }
@Override
    public void periodic() {
        BaseStatusSignal.refreshAll(angVelocity, voltage);
        DogLog.log("Feeder Roller/Angular Velociy", getAngularVelocity());
        DogLog.log("Feeder Roller/Motor Voltage", getMotorVoltage());
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

     public Command drive() {
        return startEnd(()->{
        feedermotor.setControl(new VelocityVoltage(30));   
        }, ()->{
        feedermotor.setControl(new CoastOut());
        });
     }
    public Command slowDrive() {
        return startEnd(()->{
        feedermotor.setControl(new VelocityVoltage(15));   
        }, ()->{
        feedermotor.setControl(new CoastOut());
        });
     }
}
