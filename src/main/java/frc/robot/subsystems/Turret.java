package frc.robot.subsystems;

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

/**
 * Class defines the status, velocity, voltage, and current of the intake subsystem
 */
public class Turret extends SubsystemBase {

  private TalonFX turretMotor;
  private StatusSignal<AngularVelocity> turretVelocity;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Voltage> appliedVoltageSignal;
  private StatusSignal<Angle> positionSignal;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private TurretSim simulation;
//private double RPM;
    /**
     * Initializes the motor and status signal
     */
  public Turret() {
//constructer --> initiallize
    this.turretMotor = new TalonFX(2, new CANBus("drivetrain"));
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.CurrentLimits.StatorCurrentLimitEnable = false;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
    configuration.Slot0.kP = 1.75;
    //1.75 allows the motor to reach 1 rotations with little overshoot
    turretMotor.getConfigurator().apply(configuration);
    /**
     * initializes variables for the class Turret
     */

    this.turretVelocity = turretMotor.getVelocity();
    this.statorCurrentSignal = turretMotor.getStatorCurrent();
    this.supplyCurrentSignal = turretMotor.getSupplyCurrent();
    this.appliedVoltageSignal = turretMotor.getMotorVoltage();
    this.positionSignal = turretMotor.getPosition();
    this.accelerationSignal = turretMotor.getAcceleration();

    if (RobotBase.isSimulation()){
        simulation = new TurretSim(turretMotor);
    }
  }
  /**
   * sets TurretSim as motor parameter
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduled run

    if(simulation != null){
        simulation.update();
    }

    BaseStatusSignal.refreshAll(statorCurrentSignal, supplyCurrentSignal, appliedVoltageSignal, turretVelocity, positionSignal);
    DogLog.log("Turret/Velocity", getTurretVelocity());
    DogLog.log("Turret/StatorCurrent", getStatorCurrent());
    DogLog.log("Turret/Supply Current", getSupplyCurrent());
    DogLog.log("Turret/Motor Voltage", getAppliedVoltage());
    DogLog.log("Turret/Angle", getPosition());
    DogLog.log("Turret/Angular Acceleration", getAcceleration());

  }

  //Command that makes the turret move to a certain angle
  public Command moveTo(Angle target){ //Defines the target angle command
    return startEnd(() -> { //-> defines a local function
        turretMotor.setControl(new PositionVoltage(target)); //finds voltage amount to achieve the target angle and moves there
    },() -> {
        turretMotor.setControl(new PositionVoltage(getPosition())); //ends sequence at the achieved position
    });
  }
 
  /**
   * Gets value of angular velocity of the turret
   */
  public AngularVelocity getTurretVelocity(){
    return BaseStatusSignal.getLatencyCompensatedValue(
        turretVelocity, accelerationSignal
    );
  }

  /**
   * The method takes in the status signals and returns it as a Current
   */
  public Current getStatorCurrent(){
    return statorCurrentSignal.getValue();
  }

  /**
   * The method takes in the status signals and returns it as a Current
   */
  public Current getSupplyCurrent(){
    return supplyCurrentSignal.getValue();
  }

  /**
   * The method takes in the status signals and returns it as a Voltage
   */
  public Voltage getAppliedVoltage(){
    return appliedVoltageSignal.getValue();
  }

  /**
   * The method takes in the status signals and returns it as a Angle
   */
  public Angle getPosition(){
    return MeasureUtil.toAngle(
        BaseStatusSignal.getLatencyCompensatedValue(
            positionSignal, turretVelocity
            )
    );
  }

  /**
   * The method takes in the status signals and returns it as an Angular Acceleration
   */
  public AngularAcceleration getAcceleration(){
    return accelerationSignal.getValue();
  }
}

//      _______
//    /////\\\\\\      S   M
//   //{ <>  <> }\\    U   U
//  ///|   /.  |\\\    N   N
// /////\  -  /\\\\\   N   N
//      /     \        Y   Y