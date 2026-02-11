package frc.robot.subsystems.hopper.kicker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;

public class Kicker extends SubsystemBase {
    private TalonFX kickerMotor;
    private StatusSignal AppliedVoltagesignal;
    private StatusSignal StatorCurrentSignal;
    private StatusSignal SupplyCurrent; 
    private StatusSignal MotorVelocity;

    public Kicker() {
        this.kickerMotor = new TalonFX(HopperConstants.KICKER_DEVICE_ID);
  
        
    }
    
}
