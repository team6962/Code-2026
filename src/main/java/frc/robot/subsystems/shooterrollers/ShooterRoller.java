package frc.robot.subsystems.shooterrollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//creates status signals

//REMEMBER: The build team for some unexplainable reason will use TWO motors for the shooter roller
//rewrite code to work for two motors
//why did the build team decide to do this
//remember: ask shooter roller guy on build to rethink their design choices
//update: nevermind it ended up being less work because of the follower class
//though still everyone told me that it was one motor
//wouldn't 2 make the math harder?

public class ShooterRoller extends SubsystemBase {
    private TalonFX shooterRollerMotor1;
    private TalonFX shooterRollerMotor2;
    private StatusSignal<AngularVelocity> angVelocity;
    private StatusSignal<Voltage> voltage;
    
    


    // i don't know what this does ether, probably some safety feature
    //update: it's apparently a feed forward thing to tell motors to get from point a to point b
    //new goal: find a way to delete without causing error to reduce processing power
    public ShooterRoller() {
        shooterRollerMotor1 = new TalonFX(20,new CANBus("drive train"));
        //note: device id is temporary, find out which device id will be used for shooter rollers
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
        shooterRollerMotor2 = new TalonFX(21,new CANBus("drive train"));
        //note: device id is temporary, find out which device id will be used for shooter rollers
        shooterRollerMotor2.getConfigurator().apply(
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
        shooterRollerMotor2.setControl(new Follower(shooterRollerMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
        
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


