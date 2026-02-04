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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
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
    private StatusSignal<AngularVelocity> angVelocitySignal;
    private StatusSignal<AngularAcceleration> angAccelerationSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Current> statorCurrentSignal;
    private StatusSignal<Voltage> voltageSignal;
    private ShooterRollerSim simulation;
    
    private final DoubleSubscriber velocityInput;


    // i don't know what this does ether, probably some safety feature
    //update: it's apparently a feed forward thing to tell motors to get from point a to point b
    //new goal: find a way to delete without causing error to reduce processing power

    public ShooterRoller() {
        shooterRollerMotor1 = new TalonFX(ShooterRollerConstants.MOTOR_CAN_ID_1,new CANBus("subsystem"));
        //note: device id is temporary, find out which device id will be used for shooter rollers
        //update: fixed, device id is what it should be according to tech binder
        

            
        shooterRollerMotor1.getConfigurator().apply(ShooterRollerConstants.MOTOR_CONFIGURATION);
        shooterRollerMotor2 = new TalonFX(ShooterRollerConstants.MOTOR_CAN_ID_2,new CANBus("subsystem"));
        //note: device id is temporary, find out which device id will be used for shooter rollers
        //update: fixed, device id is what it should be according to tech binder
        shooterRollerMotor2.getConfigurator().apply(ShooterRollerConstants.MOTOR_CONFIGURATION);
        // defines the variables we are keeping track of
        angVelocitySignal = shooterRollerMotor1.getVelocity();
        voltageSignal = shooterRollerMotor1.getMotorVoltage();
        angAccelerationSignal = shooterRollerMotor1.getAcceleration();
        supplyCurrentSignal = shooterRollerMotor1.getSupplyCurrent();
        statorCurrentSignal = shooterRollerMotor1.getStatorCurrent();
        velocityInput = DogLog.tunable("shooterRoller / input velocity", 0.0, newVelocity -> {
            shoot(newVelocity).schedule();
        });

        shooterRollerMotor2.setControl(new Follower(shooterRollerMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
        if (RobotBase.isSimulation()) {
            simulation = new ShooterRollerSim(shooterRollerMotor1);
        }
    }
    //makes the motor go
    public Command shoot(double targetVelocity){
        /**
         * positive voltage makes CAN ID 21 go counter clockwise, which is not what we want, please put a negative value so it goes clockwise since thats what is intended
         */
        return startEnd(()->{
            //defines a local function to set motor voltage to make it go brrrrrrrrrrrrrrrrrrrrr
            shooterRollerMotor1.setControl(new VelocityVoltage(targetVelocity));

        }, () -> {
            //defines a local function to stop motor
            shooterRollerMotor1.setControl(new CoastOut ());
        });
    }

    @Override


    public void periodic() {
        if(simulation != null) {
            simulation.update();
        }

        //this will log stuff every once in a while
        BaseStatusSignal.refreshAll(angVelocitySignal,voltageSignal,supplyCurrentSignal,statorCurrentSignal,angAccelerationSignal);
        DogLog.log("shooterRoller/voltage", getMotorVoltage());
        DogLog.log("shooterRoller/angular velocity", getAngularVelocity());
        DogLog.log("shooterRoller/statorCurrent", getStatorCurrent());
        DogLog.log("shooterRoller/angular acceleration", getAngularAcceleration());
        DogLog.log("shooterRoller/supply current", getSupplyCurrent());

    }

    // gets the angular velocity
    public AngularVelocity getAngularVelocity() {
        return angVelocitySignal.getValue();
    
    }

    public AngularAcceleration getAngularAcceleration() {
        return angAccelerationSignal.getValue();
    
    }

    public Current getSupplyCurrent() {
        return supplyCurrentSignal.getValue();
    
    }

    public Current getStatorCurrent() {
        return statorCurrentSignal.getValue();
    
    }

    //gets the voltage
    public Voltage getMotorVoltage() {
        return voltageSignal.getValue();
    
    }

}


