package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{
   private TalonFX motor;
   private CANdi candi;
   private StatusSignal<AngularAcceleration> accelerationSignal;
   private StatusSignal<AngularVelocity> velocitySignal;
   private StatusSignal<Angle> positionSignal;
   private StatusSignal<Voltage> voltageSignal;
   private StatusSignal<Current> statorCurrentSignal;
   private StatusSignal<Current> supplyCurrentSignal;
   private StatusSignal<Boolean> hallSignal;
   private ClimbSim simulation;

   public Climb(){
      motor = new TalonFX(30);
      
      TalonFXConfiguration configuration = new TalonFXConfiguration();
      configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      configuration.CurrentLimits.StatorCurrentLimitEnable = true;
      configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10; //change later
      configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
      configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      //These values are dummy values and are not to be used. Change later.
      configuration.Slot0.kP = 2.4;
      configuration.Slot0.kI = 0.0;
      configuration.Slot0.kD = 0.0;
      configuration.Slot0.kG = 0.5;

      configuration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      configuration.MotionMagic.MotionMagicCruiseVelocity = 80.0;
      configuration.MotionMagic.MotionMagicAcceleration = 160.0;
      configuration.MotionMagic.MotionMagicJerk = 1000.0;

      motor.getConfigurator().apply(configuration);

      candi = new CANdi(30);
      accelerationSignal = motor.getAcceleration();
      velocitySignal = motor.getVelocity();
      positionSignal = motor.getPosition();
      voltageSignal = motor.getMotorVoltage();
      statorCurrentSignal = motor.getStatorCurrent();
      supplyCurrentSignal = motor.getSupplyCurrent();
      hallSignal = candi.getS1Closed();
      if (RobotBase.isSimulation()) {
         simulation = new ClimbSim(motor);
      }
   }

   public LinearAcceleration getAcceleration(){
      return MetersPerSecondPerSecond.of(accelerationSignal.getValueAsDouble());
   }
   
   public LinearVelocity getVelocity(){
      return MetersPerSecond.of(velocitySignal.getValueAsDouble());
   }

   public Distance getPosition(){
      return Meters.of(positionSignal.getValueAsDouble());
   }

   public Voltage getVoltage(){
      return voltageSignal.getValue();
   }

   public Current getStatorCurrent(){
      return statorCurrentSignal.getValue();
   }

   public Current getSupplyCurrent(){
      return supplyCurrentSignal.getValue();
   }

   public boolean getHall(){
      return hallSignal.getValue();
   }

   @Override
   public void periodic() {
      BaseStatusSignal.refreshAll(accelerationSignal, velocitySignal, positionSignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal, hallSignal);
      DogLog.log("Climb/Acceleration", getAcceleration());
      DogLog.log("Climb/Velocity", getVelocity());
      DogLog.log("Climb/Position", getPosition());
      DogLog.log("Climb/Voltage", getVoltage());
      DogLog.log("Climb/StatorCurrent", getStatorCurrent());
      DogLog.log("Climb/SupplyCurrent", getSupplyCurrent());
      DogLog.log("Climb/HallSignal", getHall());
   }
}