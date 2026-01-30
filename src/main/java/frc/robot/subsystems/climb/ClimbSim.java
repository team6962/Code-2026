package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbSim {
    private TalonFXSimState climbMotor;
    private ElevatorSim physicsSim;
    public ClimbSim(TalonFX motor){
        climbMotor = motor.getSimState();
        physicsSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(1),
                Units.lbsToKilograms(10),
                Units.inchesToMeters(1),
                10.0
            ),
            DCMotor.getKrakenX60Foc(1),
            0,
            1,
            false,
            0);
    }
    public void update(){
        climbMotor.setSupplyVoltage(12);
        physicsSim.setInputVoltage(climbMotor.getMotorVoltage());
        climbMotor.setRawRotorPosition(Radians.of(physicsSim.getPositionMeters()/Units.inchesToMeters(1)*10));
        climbMotor.setRotorVelocity(RadiansPerSecond.of(physicsSim.getVelocityMetersPerSecond()/Units.inchesToMeters(1)*10));
        climbMotor.setRotorAcceleration(RadiansPerSecondPerSecond.of(physicsSim.getVelocityMetersPerSecond()/Units.inchesToMeters(1)*10));

    }
    
}
