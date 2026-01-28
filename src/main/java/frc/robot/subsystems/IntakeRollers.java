// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.phoenix.StatusUtil;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
//   private String state;
//   private int speed;
    private TalonFX intakeMotor; 
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Current> statorCurrentSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Voltage> appliedVoltageSignal;
/**
 * intializes motor and status signals
 * Class for Intake Rollers
 */
    public IntakeRollers() {
        this.intakeMotor = new TalonFX(1, new CANBus("canivore")); //temporary
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.CurrentLimits.StatorCurrentLimitEnable = false;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
        configuration.Slot0.kV = 0.12;
        intakeMotor.getConfigurator().apply(configuration);
        this.velocitySignal = intakeMotor.getVelocity();
        this.statorCurrentSignal = intakeMotor.getStatorCurrent();
        this.supplyCurrentSignal = intakeMotor.getSupplyCurrent();
        this.appliedVoltageSignal = intakeMotor.getMotorVoltage();
      
    }
    /** 
     * Returns command to make the motor move and stop
    */
    
    
    public Command move() {
      //if (state == "normal") {
        return startEnd(( ) -> {
          intakeMotor.setControl(new VelocityVoltage(-50));
          
        }, () -> {
          intakeMotor.setControl(new CoastOut());
        });
        //}
        //else if (state == "stuck") {
        //return startEnd(( ) -> {
        //   intakeMotor.setControl(new VelocityVoltage(50));
          
        // }, () -> {
        //   intakeMotor.setControl(new CoastOut());
        // });
        //}
    }

    public AngularVelocity getVelocity() {
      return velocitySignal.getValue();
    }

    public Current getStatorCurrent() {
      return statorCurrentSignal.getValue();
    }

    public Current getSupplyCurrent() {
      return supplyCurrentSignal.getValue();
    }

    public Voltage getAppliedVoltage() {
      return appliedVoltageSignal.getValue();
    }
    
    @Override
    public void periodic() {
        DogLog.log("intakeRollers/velocity",getVelocity());
        DogLog.log("intakeRollers/statorCurrent", getStatorCurrent());
        DogLog.log("intakeRollers/SupplyCurrent", getSupplyCurrent());
        DogLog.log("intakeRollers/appliedVoltage", getAppliedVoltage());
        StatusUtil.check(BaseStatusSignal.refreshAll(velocitySignal,statorCurrentSignal,supplyCurrentSignal,appliedVoltageSignal));
    }

} 
