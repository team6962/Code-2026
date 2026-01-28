// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.math.MeasureUtil;
import com.team6962.lib.phoenix.StatusUtil;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
//motor
    private TalonFX turretMotor; 
// Status Signals
    private StatusSignal <Angle> angleSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Current> statorCurrentSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Voltage> appliedVoltageSignal;
    private StatusSignal<AngularAcceleration> angularAccelerationSignal;
    private TurretSim simulation;
    /**
     * initializes variables in turret class
     */
    public Turret() {
        this.turretMotor = new TalonFX(2, new CANBus("drivetrain")); //temporary
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.CurrentLimits.StatorCurrentLimitEnable = false;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
        configuration.Slot0.kP = 1.3;
        turretMotor.getConfigurator().apply(configuration);
        this.velocitySignal = turretMotor.getVelocity();
        this.angleSignal = turretMotor.getPosition();
        this.statorCurrentSignal = turretMotor.getStatorCurrent();
        this.supplyCurrentSignal = turretMotor.getSupplyCurrent();
        this.appliedVoltageSignal = turretMotor.getMotorVoltage();
        this.angularAccelerationSignal = turretMotor.getAcceleration();

        if (RobotBase.isSimulation()) {
            simulation = new TurretSim(turretMotor);
        }
    
    
    }
    /**
     * Creates command to move turret to given target angle
     * @param target (target angle)
     * @return Command
     */
    public Command moveTo(Angle target) {
        return startEnd(( ) -> {
          turretMotor.setControl(new PositionVoltage(target));

          
        }, () -> {
          turretMotor.setControl(new PositionVoltage(getPosition()));
        });
        
    }
   
/**
 * Gets status signal and converts to Current variable type
 * @return Current
 */
    public Current getStatorCurrent() {
      return statorCurrentSignal.getValue();
    }
/**
 * Gets status signal and converts to Current variable type
 * @return Current
 */
    public Current getSupplyCurrent() {
      return supplyCurrentSignal.getValue();
    }

/**
 * Gets status signal and converts to Voltage variable type
 * @return Voltage
 */
    public Voltage getAppliedVoltage() {
        return appliedVoltageSignal.getValue();
    }
/**
 * Gets status signal and converts to Angular Acceleration variable type
 * @return AngularAcceleration
 */
    public AngularAcceleration getAngularAcceleration() {
        return angularAccelerationSignal.getValue();
    }
/**
 * Converts status signal to angle with latency compensation
 * @return Angle
 */
    public Angle getPosition() {
        return MeasureUtil.toAngle(
            BaseStatusSignal.getLatencyCompensatedValue(
                angleSignal,velocitySignal
                )
            );
    }
/**
 * Converts status signal to Angular Velocity with latency compensation
 * @return Angular Velocity
 */
    public AngularVelocity getVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(
            velocitySignal,angularAccelerationSignal
        );
    }

  @Override
  /**
   * updates periodically
   * and logs values in DogLog
   */
  public void periodic() {
        if (simulation != null) {
            simulation.update();
        }
        
        DogLog.log("turret/velocity",getVelocity());
        DogLog.log("turret/statorCurrent", getStatorCurrent());
        DogLog.log("turret/supplyCurrent", getSupplyCurrent());
        DogLog.log("turret/appliedVoltage", getAppliedVoltage());
        DogLog.log("turret/angle", getPosition() );
        DogLog.log("turret/angularAcceleration", getAngularAcceleration() );
        //refresh every status signal
        StatusUtil.check(BaseStatusSignal.refreshAll(velocitySignal,statorCurrentSignal,supplyCurrentSignal,appliedVoltageSignal,angularAccelerationSignal,angleSignal));
  }
}



