package frc.robot.subsystems.hopper.sensors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;

import edu.wpi.first.units.measure.Angle;

public class HopperSensors {
    private TalonFX motor;

    private CANrange candi;
    
    // private StatusSignal<> positionSignal;
}
