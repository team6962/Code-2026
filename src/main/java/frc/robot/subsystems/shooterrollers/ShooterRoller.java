package frc.robot.subsystems.shooterrollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Scanner; // Import the Scanner class

//creates status signals
public class ShooterRoller extends SubsystemBase {
    private TalonFX shooterRollerMotor1;
    private StatusSignal<AngularVelocity> angVelocity;
    private StatusSignal<Voltage> voltage;
    


    // i don't know what this does ether, probably some safety feature
    public ShooterRoller() {
        shooterRollerMotor1 = new TalonFX(2,new CANBus("drive train"));
        shooterRollerMotor1.getConfigurator().apply(
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
        
        // defines the variables we are keeping track of
        angVelocity = shooterRollerMotor1.getVelocity();
        voltage = shooterRollerMotor1.getMotorVoltage();
        
    }
    //makes the motor go
    public Command shoot(){
        return startEnd(()->{
            //defines a local function to set motor voltage to make it go brrrrrrrrrrrrrrrrrrrrr
            shooterRollerMotor1.setControl(new PositionVoltage(1.0));

        }, () -> {
            //defines a local function to stop motor
            shooterRollerMotor1.setControl(new CoastOut ());
        });
    }

    @Override


    public void periodic() {
        //if(simulation != null) {
        //    simulation.update();
        //}

        //this will log stuff every once in a while
        BaseStatusSignal.refreshAll(angVelocity,voltage);
        DogLog.log("outtake/voltage", getMotorVoltage());
        DogLog.log("angular velocity", getAngularVelocity());

    }

    // gets the angular velocity
    public AngularVelocity getAngularVelocity() {
        return angVelocity.getValue();
    
    }

    //gets the voltage
    public Voltage getMotorVoltage() {
        return voltage.getValue();
    
    }

}


