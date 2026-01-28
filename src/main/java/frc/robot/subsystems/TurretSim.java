package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretSim {
    private TalonFXSimState motorSim;
    private DCMotorSim physicsSim;
    private AngularVelocity lastVelocity = RadiansPerSecond.of(0);
    
    public TurretSim(TalonFX motor) {
        motorSim = motor.getSimState();
        physicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                0.000174,
                42.0
            ),
            DCMotor.getKrakenX60Foc(1)
        );
        
    }
    private static double invert(double value, boolean shouldBeInverted) {
        return shouldBeInverted ? -value : value;
    }
    private static Angle invert(Angle angle, boolean shouldBeInverted) {
        return shouldBeInverted ? angle.unaryMinus() : angle;
    }
    private static AngularVelocity invert(AngularVelocity velocity, boolean shouldBeInverted) {
        return shouldBeInverted ? velocity.unaryMinus() : velocity;
    }

    public void update(){
        
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double motorVoltage = invert(motorSim.getMotorVoltage(), false);
        physicsSim.setInput(motorVoltage);
        physicsSim.update(0.02);
        Angle position = Radians.of(physicsSim.getAngularPositionRad());
        AngularVelocity velocity = RadiansPerSecond.of(physicsSim.getAngularVelocityRadPerSec());
        motorSim.setRawRotorPosition(
            invert(position, false)
                .times(42.0)
        );
        motorSim.setRotorVelocity(
            invert(velocity, false)
                .times(42.0)
        );
    }
   
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    HEALTH * * *            WORLD 1-1          //     HEALTH * *              WORLD 1-2         //     HEALTH *               WORLD 1-3
//_______________________________________________//_______________________________________________//_________________________________________________
////////////////////  /   /  ////////////////////////////////////////////////// /////////////////////////////////////////////////////////////////////
///                                     [[__]]  ///              /        __ _[]_ ___[.    ]      
///            .       *%&               |__|   ///             /        /  q/. .\p  ////.|       //////////////////////////////////////
//                     [_]|             [|__|]  ///       []   /        ||  [- ]_/   ///. |       //                                        -==
//                     |  \              \--/           /[_]-=*         ||i__/   |   ////.|       //      [     ]    [     ]    [     ]     L[]
//                                                       | |   \        ||  | ___ |  / /. |       //       | 1 |      | 2 |      | 3 |      _[_]\
//     .                [] /                                    \        \\  |   |   / /. |       //       |   |      |   |      |   |          - 
///                    /[_]            .        ///              \        \==========/.   |       //________________________________________________
///                     | |                     ///                                 [.    ]       //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///        A FUNGUS-RIDDEN WORLD                ///             THE ABANDONED PIZZAPLEX           ///                   WARP ZONE
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// *The Infected Approaches you |              // * You hear footsteps in the     |               // * Welcome to Warp Zone! 
///                              | Bandages:    // left door                       | Time:         //
/// 1. Attack                    |  / / /       // 1. Check Cameras                | 5:59 A.M.     //
/// 2. Flee                      | Ammunition:  // 2. Doors                        | Power:        //
///                              |  II II II    //                                 | 39%           //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                       //   //
///                                                                                                       //   // 
////////////////////////////////////////////////////////////////////////////////////////////////////////////   ///////////////////////////////////////
/// 
/// 
/// 
/// 
/// 
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~///
/// 
///     *                                        *                              
/// 
///                *                                            *
/// 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////