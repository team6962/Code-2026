package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.MeasureUtil;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANdi candi; // CANbus name may need to be changed later

  private final StatusSignal<Angle> angle;
  private final StatusSignal<AngularVelocity> angVelocity;
  private final StatusSignal<AngularAcceleration> angAcceleration;

  /**
   * Status signal for the profile reference (the intermediate target that the motion profile thinks
   * the hood should be at) in rotations
   */
  private final StatusSignal<Double> profileReferenceSignal;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> current;

  private final StatusSignal<Boolean> candiTriggeredSignal;

  private ShooterHoodSim simulation;
  private boolean isZeroed = false;

  /** Initializes the motor and status signal */
  public ShooterHood() {
    hoodMotor =
        new TalonFX(ShooterHoodConstants.MOTOR_CAN_ID, new CANBus(ShooterHoodConstants.CANBUS));
    candi = new CANdi(ShooterHoodConstants.CANDI_CAN_ID, new CANBus(ShooterHoodConstants.CANBUS));

    hoodMotor.getConfigurator().apply(ShooterHoodConstants.MOTOR_CONFIGURATION);

    angVelocity = hoodMotor.getVelocity();
    voltage = hoodMotor.getMotorVoltage();
    angAcceleration = hoodMotor.getAcceleration();
    angle = hoodMotor.getPosition();
    current = hoodMotor.getSupplyCurrent();
    profileReferenceSignal = hoodMotor.getClosedLoopReference();

    CANdiConfiguration candiConfig = new CANdiConfiguration();
    candi.getConfigurator().apply(candiConfig);
    candiTriggeredSignal = candi.getS2Closed();

    DogLog.tunable(
        "Hood Motor/Hood Target Angle (Degrees)",
        0.0,
        newAngle -> {
          setHoodAngle(Degrees.of(newAngle)).schedule();
        });

    if (RobotBase.isSimulation()) {
      simulation = new ShooterHoodSim(hoodMotor);
    }
    hoodMotor.setPosition(ShooterHoodConstants.MAX_ANGLE);
  }

  @Override
  public void periodic() {
    // In disabled mode, set the motor's target position to the current position
    // This ensures that the motor will try to stay in place when re-enabled
    if (RobotState.isDisabled()) {
      setPositionControl(getPosition());
    }

    if (simulation != null) {
      simulation.update();
    }

    BaseStatusSignal.refreshAll(
        angVelocity, voltage, angAcceleration, angle, current, profileReferenceSignal);
    DogLog.log("Hood Motor/Angle", getPosition().in(Degrees));
    DogLog.log("Hood Motor/Angular Velocity", getAngularVelocity());
    DogLog.log("Hood Motor/Angular Acceleration", getAngularAcceleration());
    DogLog.log("Hood Motor/Motor Voltage", getMotorVoltage());
    DogLog.log("Hood Motor/Current", getSupply());
    DogLog.log(
        "Hood Motor/Profile Reference Angle",
        Rotations.of(profileReferenceSignal.getValue()).in(Degrees),
        Degrees);

    LoggingUtil.log("Hood Motor/Control Request", hoodMotor.getAppliedControl());

    // Update the currently applied control request with a new gravity feedforward
    // voltage if the control request is a MotionMagicVoltage
    if (hoodMotor.getAppliedControl() instanceof MotionMagicVoltage motionMagicControlRequest) {
      setPositionControl(motionMagicControlRequest.getPositionMeasure());
    }

    if (isCandiTriggered() && getPosition().lt(ShooterHoodConstants.MIN_ANGLE)) {
      hoodMotor.setPosition(ShooterHoodConstants.MIN_ANGLE);
      isZeroed = true;
    }
  }

  public Angle clampPositionToSafeRange(Angle input) {
    if (input.gt(ShooterHoodConstants.MAX_ANGLE)) {
      return ShooterHoodConstants.MAX_ANGLE;
    } else if (input.lt(ShooterHoodConstants.MIN_ANGLE)) {
      return ShooterHoodConstants.MIN_ANGLE;
    }
    return input;
  }

  /** Gets Angular Velocity and Motor Voltage */
  public Angle getPosition() {
    return MeasureUtil.toAngle(BaseStatusSignal.getLatencyCompensatedValue(angle, angVelocity));
  }

  public AngularAcceleration getAngularAcceleration() {
    return angAcceleration.getValue();
  }

  public AngularVelocity getAngularVelocity() {
    return BaseStatusSignal.getLatencyCompensatedValue(angVelocity, angAcceleration);
  }

  public Voltage getMotorVoltage() {
    return voltage.getValue();
  }

  public Current getSupply() {
    return current.getValue();
  }

  public boolean isCandiTriggered() {
    return candiTriggeredSignal.getValue();
  }

  public Command setHoodAngle(Angle targetAngle) {
    Angle clampedAngle = clampPositionToSafeRange(targetAngle);

    DogLog.log("Hood Motor/Target Angle", clampedAngle.in(Degrees));

    return startEnd(
        () -> {
          setPositionControl(clampedAngle);
        },
        () -> {
          setPositionControl(getPosition());
        });
  }

  public Command setVoltage(Voltage voltage) {
    return runEnd(
        () -> {
          if (isZeroed) {
            hoodMotor.setControl(
                new VoltageOut(
                    voltage.plus(
                        Volts.of(Math.cos(getPosition().in(Radians)) * ShooterHoodConstants.kG))));
          } else {
            hoodMotor.setControl(new NeutralOut());
          }
        },
        () -> {
          setPositionControl(getPosition());
        });
  }

  /**
   * Applies a control request to move to a specific position with a MotionMagicVoltage control
   * request, including an additional gravity feedforward. The gravity feedforward will be
   * automatically updated in periodic().
   *
   * @param position The target position to move to.
   */
  private void setPositionControl(Angle position) {
    if (isZeroed) {
      hoodMotor.setControl(
          new MotionMagicVoltage(position)
              .withFeedForward(Math.cos(getPosition().in(Radians)) * ShooterHoodConstants.kG));
    } else {
      hoodMotor.setControl(new NeutralOut());
    }
  }
}
