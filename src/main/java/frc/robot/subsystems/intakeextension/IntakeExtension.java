package frc.robot.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This subsystem controls the extension of the intake out of the robot and back in. */
public class IntakeExtension extends SubsystemBase {

  private TalonFX motor;
  private CANdi candi;

  // The status signals' rotations actually represent meters of mechanism motion
  private StatusSignal<Angle> positionSignal;
  private StatusSignal<AngularVelocity> linearVelocitySignal;
  private StatusSignal<AngularAcceleration> linearAccelerationSignal;

  private StatusSignal<Voltage> voltageSignal;

  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;

  private StatusSignal<Boolean> hallSensorTriggeredSignal;

  private StatusSignal<Double> closedLoopReferenceSignal;

  private IntakeExtensionSim simulation;

  public boolean isZeroed = false;

  public IntakeExtension() {
    motor = new TalonFX(IntakeExtensionConstants.MOTOR_CAN_ID, new CANBus("subsystems"));
    motor.getConfigurator().apply(IntakeExtensionConstants.MOTOR_CONFIGURATION);

    candi = new CANdi(IntakeExtensionConstants.CANDI_DEVICE_ID, new CANBus("subsystems"));
    candi.getConfigurator().apply(IntakeExtensionConstants.CANDI_CONFIGURATION);

    positionSignal = motor.getPosition();
    linearVelocitySignal = motor.getVelocity();
    linearAccelerationSignal = motor.getAcceleration();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();
    hallSensorTriggeredSignal = candi.getS1Closed();
    closedLoopReferenceSignal = motor.getClosedLoopReference();

    if (RobotBase.isSimulation()) {
      simulation = new IntakeExtensionSim(motor);
      isZeroed = true;
    } else {
      motor.setPosition(IntakeExtensionConstants.MIN_POSITION.in(Meters));
    }
  }

  /**
   * Extends the intake outwards until it reaches the maximum position, then holds it there. Only
   * runs if the intake is zeroed, which happens when the CANdi is triggered.
   *
   * @return A command that extends the intake.
   */
  public Command extend() {
    return startEnd(
            () -> {
              motor.setControl(
                  new MotionMagicVoltage(IntakeExtensionConstants.MAX_POSITION.in(Meters)));
            },
            () -> {
              motor.setControl(new MotionMagicVoltage(getPosition().in(Meters)));
            })
        .until(
            () ->
                getPosition()
                    .isNear(
                        IntakeExtensionConstants.MAX_POSITION,
                        IntakeExtensionConstants.POSITION_TOLERANCE))
        .onlyIf(() -> isZeroed);
  }

  /**
   * Retracts the intake inwards until it reaches the minimum position, then holds it there.
   *
   * @return A command that retracts the intake.
   */
  public Command retract() {
    return startEnd(
            () -> {
              motor.setControl(
                  new MotionMagicVoltage(IntakeExtensionConstants.MIN_POSITION.in(Meters)));
            },
            () -> {
              motor.setControl(new MotionMagicVoltage(getPosition().in(Meters)));
            })
        .until(
            () ->
                getPosition()
                    .isNear(
                        IntakeExtensionConstants.MIN_POSITION,
                        IntakeExtensionConstants.POSITION_TOLERANCE));
  }

  /**
   * Finds the linear velocity of the end of the intake. Positive values mean that the intake is
   * extending outwards, and negative values mean that the intake is retracting inwards.
   *
   * @return The linear velocity as a LinearVelocity object.
   */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(
        BaseStatusSignal.getLatencyCompensatedValue(linearVelocitySignal, linearAccelerationSignal)
            .in(RotationsPerSecond));
  }

  /**
   * Finds the position of the intake.
   *
   * @return The position as a Distance object.
   */
  public Distance getPosition() {
    return Meters.of(
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, linearVelocitySignal)
            .in(Rotations));
  }

  /**
   * Finds the linear acceleration of the end of the intake.
   *
   * @return The linear acceleration as a LinearAcceleration object.
   */
  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(linearAccelerationSignal.getValueAsDouble());
  }

  /**
   * Voltage finds the voltage signal.
   *
   * @return It returns the voltage signal.
   */
  public Voltage getVoltage() {
    return voltageSignal.getValue();
  }

  /**
   * This method finds the stator current.
   *
   * @return It returns the current.
   */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /**
   * This finds the supply current.
   *
   * @return It returns the supply current.
   */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  /**
   * This finds out whether the hall sensor is triggered, which happens when the intake is fully
   * retracted.
   *
   * @return true if the hall sensor is triggered, and false otherwise.
   */
  private boolean isHallSensorTriggered() {
    return hallSensorTriggeredSignal.getValue();
  }

  /**
   * This finds the closed loop reference, which is the current position where the motion profile
   * expects the intake to be.
   *
   * @return The closed loop reference
   */
  private Distance getClosedLoopReference() {
    return Meters.of(closedLoopReferenceSignal.getValue());
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }

    if (isHallSensorTriggered() && getPosition().lt(IntakeExtensionConstants.MIN_POSITION)) {
      motor.setPosition(IntakeExtensionConstants.MIN_POSITION.in(Meters));
      isZeroed = true;
    }

    BaseStatusSignal.refreshAll(
        linearVelocitySignal,
        voltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        positionSignal,
        linearAccelerationSignal,
        hallSensorTriggeredSignal,
        closedLoopReferenceSignal);

    DogLog.log("Intake/Position", getPosition());
    DogLog.log("Intake/Velocity", getVelocity());
    DogLog.log("Intake/Acceleration", getAcceleration());
    DogLog.log("Intake/Voltage", getVoltage());
    DogLog.log("Intake/StatorCurrent", getStatorCurrent());
    DogLog.log("Intake/SupplyCurrent", getSupplyCurrent());
    DogLog.log("Intake/HallSensorTriggered", isHallSensorTriggered());
    DogLog.log("Intake/ClosedLoopReference", getClosedLoopReference());
  }
}
