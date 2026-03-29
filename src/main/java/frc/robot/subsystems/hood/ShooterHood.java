package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.CurrentDrawLogger;
import com.team6962.lib.math.MeasureUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class ShooterHood extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANdi candi;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;

  /**
   * Status signal for the profile reference (the intermediate target that the motion profile thinks
   * the hood should be at) in rotations
   */
  private final StatusSignal<Double> profileReferenceSignal;

  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Boolean> hallSensorTriggeredSignal;

  private ShooterHoodSim simulation;
  private boolean isZeroed = false;
  private double kG = ShooterHoodConstants.kG;

  private Supplier<Boolean> shouldLowerHoodSupplier;

  private Angle manualOffset = Degrees.of(0);

  /** Initializes the motor and status signal */
  public ShooterHood(Supplier<Boolean> shouldLowerHoodSupplier) {
    hoodMotor =
        new TalonFX(ShooterHoodConstants.MOTOR_CAN_ID, new CANBus(ShooterHoodConstants.CANBUS));
    candi = new CANdi(ShooterHoodConstants.CANDI_CAN_ID, new CANBus(ShooterHoodConstants.CANBUS));

    this.shouldLowerHoodSupplier = shouldLowerHoodSupplier;

    if (RobotBase.isSimulation()) {
      ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kP = 80.0;
      ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kD = 3.0;
      ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kV = 0.0; // 4.3;
      ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kA = 0.0;
      ShooterHoodConstants.kG = 0.22;
      kG = ShooterHoodConstants.kG;
      ShooterHoodConstants.MOTOR_CONFIGURATION.MotionMagic.MotionMagicCruiseVelocity =
          1000.0; // 30.0;
      ShooterHoodConstants.MOTOR_CONFIGURATION.MotionMagic.MotionMagicAcceleration =
          1000.0; // 30.0;
    }

    hoodMotor.getConfigurator().apply(ShooterHoodConstants.MOTOR_CONFIGURATION);
    candi.getConfigurator().apply(ShooterHoodConstants.CANDI_CONFIGURATION);

    velocitySignal = hoodMotor.getVelocity();
    voltageSignal = hoodMotor.getMotorVoltage();
    accelerationSignal = hoodMotor.getAcceleration();
    positionSignal = hoodMotor.getPosition();
    statorCurrentSignal = hoodMotor.getStatorCurrent();
    supplyCurrentSignal = hoodMotor.getSupplyCurrent();
    hallSensorTriggeredSignal = candi.getS2Closed();
    profileReferenceSignal = hoodMotor.getClosedLoopReference();

    DogLog.tunable(
        "Hood/Hood Target Angle (Degrees)",
        0.0,
        newAngle -> {
          CommandScheduler.getInstance().schedule(moveTo(Degrees.of(newAngle)));
        });

    DogLog.tunable(
        "Hood/Hood kP",
        ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kP,
        newKP -> {
          hoodMotor
              .getConfigurator()
              .apply(ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.withKP(newKP));
        });

    DogLog.tunable(
        "Hood/Hood kD",
        ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kD,
        newKD -> {
          hoodMotor
              .getConfigurator()
              .apply(ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.withKD(newKD));
        });

    DogLog.tunable(
        "Hood/Hood kS",
        ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kS,
        newKS -> {
          hoodMotor
              .getConfigurator()
              .apply(ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.withKS(newKS));
        });

    DogLog.tunable(
        "Hood/Hood kV",
        ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kV,
        newKV -> {
          hoodMotor
              .getConfigurator()
              .apply(ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.withKV(newKV));
        });

    DogLog.tunable(
        "Hood/Hood kA",
        ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kA,
        newKA -> {
          hoodMotor
              .getConfigurator()
              .apply(ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.withKA(newKA));
        });

    DogLog.tunable(
        "Hood/Hood kG",
        kG,
        newKG -> {
          kG = newKG;
        });

    DogLog.tunable(
        "Hood/Hood Cruise Velocity",
        ShooterHoodConstants.MOTOR_CONFIGURATION.MotionMagic.MotionMagicCruiseVelocity,
        newCruiseVelocity -> {
          hoodMotor
              .getConfigurator()
              .apply(
                  ShooterHoodConstants.MOTOR_CONFIGURATION.MotionMagic
                      .withMotionMagicCruiseVelocity(newCruiseVelocity));
        });

    DogLog.tunable(
        "Hood/Hood Acceleration",
        ShooterHoodConstants.MOTOR_CONFIGURATION.MotionMagic.MotionMagicAcceleration,
        newAcceleration -> {
          hoodMotor
              .getConfigurator()
              .apply(
                  ShooterHoodConstants.MOTOR_CONFIGURATION.MotionMagic.withMotionMagicAcceleration(
                      newAcceleration));
        });

    DogLog.tunable(
        "Hood/Hood Voltage",
        0.0,
        voltage -> {
          CommandScheduler.getInstance().schedule(moveAtVoltage(Volts.of(voltage)));
        });

    if (RobotBase.isSimulation()) {
      simulation = new ShooterHoodSim(hoodMotor);
      isZeroed = true;
    } else {
      hoodMotor.setPosition(ShooterHoodConstants.MIN_ANGLE);
    }

    CurrentDrawLogger.add("Shooter Hood", this::getSupplyCurrent);
  }

  @Override
  public void periodic() {
    // In disabled mode, set the motor's target position to the current position
    // This ensures that the motor will try to stay in place when re-enabled
    if (RobotState.isDisabled()) {
      if (isZeroed) setPositionControl(getPosition());
      else
        hoodMotor.setControl(
            new CoastOut()); // Coast if not zeroed to allow manual movement for zeroing
    }

    if (simulation != null) {
      simulation.update();
    }

    BaseStatusSignal.refreshAll(
        velocitySignal,
        voltageSignal,
        accelerationSignal,
        positionSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        profileReferenceSignal,
        hallSensorTriggeredSignal);
    DogLog.log("Hood/Position", getPosition().in(Degrees), Degrees);
    DogLog.log("Hood/ManualOffsetDegs", manualOffset);
    DogLog.log("Hood/Velocity", getVelocity());
    DogLog.log("Hood/Acceleration", getAcceleration());
    DogLog.log("Hood/AppliedVoltage", getAppliedVoltage());
    DogLog.log("Hood/SupplyCurrent", getSupplyCurrent());
    DogLog.log("Hood/StatorCurrent", getStatorCurrent());
    DogLog.log("Hood/HallSensorTriggered", isHallSensorTriggered());
    DogLog.log("Hood/ShouldBeLowered", shouldLowerHoodSupplier.get());
    DogLog.forceNt.log("Hood/IsZeroed", isZeroed);
    DogLog.log(
        "Hood/ProfileReferenceAngle",
        Rotations.of(profileReferenceSignal.getValue()).in(Degrees),
        Degrees);

    // LoggingUtil.log("Hood/ControlRequest", hoodMotor.getAppliedControl());

    // Update the currently applied control request with a new gravity feedforward
    // voltage if the control request is a MotionMagicVoltage
    if (hoodMotor.getAppliedControl() instanceof MotionMagicVoltage motionMagicControlRequest) {
      setPositionControl(motionMagicControlRequest.getPositionMeasure());
    }

    if (RobotState.isDisabled()
        && isHallSensorTriggered()
        && getPosition().lt(ShooterHoodConstants.MIN_ANGLE)) {
      hoodMotor.setPosition(ShooterHoodConstants.MIN_ANGLE);
      isZeroed = true;
    }
  }

  /**
   * Clamps the given position to within the limiits of the hood.
   *
   * @param input angle to clamp
   * @return the clamped angle
   */
  public Angle clampPositionToSafeRange(Angle input) {
    if (input.gt(ShooterHoodConstants.MAX_ANGLE)) {
      return ShooterHoodConstants.MAX_ANGLE;
    } else if (input.lt(ShooterHoodConstants.MIN_ANGLE)) {
      return ShooterHoodConstants.MIN_ANGLE;
    }
    return input;
  }

  /**
   * Gets the hoods position. Increasing angles represent the hood moving upward, decreasing angles
   * represent the hood moving downward.
   *
   * @return the hoods position
   */
  public Angle getPosition() {
    return MeasureUtil.toAngle(
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal));
  }

  /**
   * Gets the hoods velocity. Positive values represent the hood moving upward, negative values
   * represent the hood moving downward.
   *
   * @return the hoods velocity
   */
  public AngularVelocity getVelocity() {
    return BaseStatusSignal.getLatencyCompensatedValue(velocitySignal, accelerationSignal);
  }

  /**
   * Gets the hoods acceleration.
   *
   * @return the hoods acceleration
   */
  public AngularAcceleration getAcceleration() {
    return accelerationSignal.getValue();
  }

  /**
   * Gets the voltage applied to the motor
   *
   * @return the applied voltage
   */
  public Voltage getAppliedVoltage() {
    return voltageSignal.getValue();
  }

  /**
   * Gets the motor supply current
   *
   * @return the supply current
   */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  /**
   * Gets the motor stator current
   *
   * @return the stator current
   */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /**
   * Gets whether the hall sensor is near its minimum position
   *
   * @return true if the hall sensor is triggered
   */
  public boolean isHallSensorTriggered() {
    return hallSensorTriggeredSignal.getValue();
  }

  public void setOffsetAngle(Angle newOffset) {
    manualOffset = newOffset;
  }
 /**
  * Manual correction to shooter hood if auto shoot is consistently off
  * @param targetAngle
  * @return
  */

  /**
   * Returns a command that moves the hood to the given target angle.
   *
   * @param targetAngle the target angle to move towards
   * @return the command that moves to the target angle
   */
  public Command moveTo(Angle targetAngle) {

    Angle clampedAngle = clampPositionToSafeRange(targetAngle);

    DogLog.log("Hood/TargetPosition", clampedAngle.in(Degrees));

    return startEnd(
        () -> {
          setPositionControl(clampedAngle);
        },
        () -> {
          setPositionControl(getPosition());
        });
  }

  /**
   * Returns a command that moves the hood to the given an inputted targetAngleSupplier.
   *
   * @param targetAngleSupplier the supplied target angle to move towards
   * @return the command that moves to the supplied target angle
   */
  public Command moveTo(Supplier<Angle> targetAngleSupplier) {
    return runEnd(
        () -> {
          Angle clampedAngle =
              clampPositionToSafeRange(targetAngleSupplier.get().plus(manualOffset));
          DogLog.log("Hood/TargetPosition", clampedAngle.in(Degrees));
          setPositionControl(clampedAngle);
        },
        () -> {
          setPositionControl(getPosition());
        });
  }

  /**
   * Moves the hood at the given voltage with additional gravity compensation applied.
   *
   * @param voltage the voltage to apply
   * @return the command that runs the hood at that voltage
   */
  public Command moveAtVoltage(Voltage voltage) {
    return runEnd(
        () -> {
          if (isZeroed) {
            hoodMotor.setControl(
                new VoltageOut(
                    voltage
                        .plus(
                            Volts.of(ShooterHoodConstants.MOTOR_CONFIGURATION.Slot0.kS)
                                .times(Math.signum(voltage.in(Volts))))
                        .plus(Volts.of(Math.cos(getPosition().in(Radians)) * kG))));
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
    if (!isZeroed) {
      hoodMotor.setControl(new NeutralOut());
    } else if (shouldLowerHoodSupplier.get()) {
      hoodMotor.setControl(
          new MotionMagicVoltage(ShooterHoodConstants.MIN_ANGLE)
              .withFeedForward(Math.cos(getPosition().in(Radians)) * kG));
    } else {
      hoodMotor.setControl(
          new MotionMagicVoltage(position)
              .withFeedForward(Math.cos(getPosition().in(Radians)) * kG));
    }
  }

  private void setPositionVelocityControl(Angle position, AngularVelocity velocity) {
    if (!isZeroed) {
      hoodMotor.setControl(new NeutralOut());
    } else if (shouldLowerHoodSupplier.get()) {
      hoodMotor.setControl(
          new MotionMagicVoltage(ShooterHoodConstants.MIN_ANGLE)
              .withFeedForward(Math.cos(getPosition().in(Radians)) * kG));
    } else {
      hoodMotor.setControl(
          new PositionVoltage(position)
              .withVelocity(velocity)
              .withFeedForward(Math.cos(getPosition().in(Radians)) * kG));
    }
  }

  /**
   * Creates a Command that moves the hood to a target angle provided by the given supplier, while
   * attempting to maintain the target velocity, which should be the velocity of the target angle.
   * This is a done using a combination of basic PID and feedforward, with motion profiling used
   * when error becomes large, ensuring fast movement when the hood reaches its limits and needs to
   * rotate 360 degrees to reach the target angle.
   *
   * @param targetAngleSupplier supplier that is sampled repeatedly to provide the desired target
   *     angle
   * @param targetVelocitySupplier supplier that is sampled repeatedly to provide the desired target
   *     velocity, which should be the velocity of the target angle; used for feedforward and can
   *     help improve tracking performance when the target angle is changing rapidly
   * @return a Command that, while scheduled, continuously moves the hood toward the supplied target
   *     angle while attempting to maintain the supplied target velocity
   */
  public Command track(
      Supplier<Angle> targetAngleSupplier, Supplier<AngularVelocity> targetVelocitySupplier) {
    Command command =
        new Command() {
          private TrapezoidProfile profile =
              new TrapezoidProfile(
                  new TrapezoidProfile.Constraints(
                      ShooterHoodConstants.MOTOR_CONFIGURATION
                          .MotionMagic
                          .MotionMagicCruiseVelocity,
                      ShooterHoodConstants.MOTOR_CONFIGURATION
                          .MotionMagic
                          .MotionMagicAcceleration));
          private TrapezoidProfile.State previousProfileState;
          private double previousUpdateTimestamp;

          @Override
          public void initialize() {
            previousProfileState =
                new TrapezoidProfile.State(
                    getPosition().in(Rotations), getVelocity().in(RotationsPerSecond));
            previousUpdateTimestamp = Timer.getFPGATimestamp();
          }

          @Override
          public void execute() {
            Angle unclampedTargetPosition = targetAngleSupplier.get().plus(manualOffset);
            AngularVelocity targetVelocity = targetVelocitySupplier.get();

            if (unclampedTargetPosition == null) return;
            if (targetVelocity == null) targetVelocity = RotationsPerSecond.of(0);

            Angle targetPosition = clampPositionToSafeRange(targetAngleSupplier.get());

            TrapezoidProfile.State profileState =
                profile.calculate(
                    Timer.getFPGATimestamp() - previousUpdateTimestamp,
                    previousProfileState,
                    new TrapezoidProfile.State(
                        targetPosition.in(Rotations),
                        0 // This can also be set to targetVelocity.in(RotationsPerSecond)
                        // if the profile should go to the target position and velocity. However, 0
                        // seems to perform better in simulation
                        ));

            previousProfileState = profileState;
            previousUpdateTimestamp = Timer.getFPGATimestamp();

            if (targetPosition.isNear(getPosition(), Degrees.of(3))) {
              setPositionVelocityControl(targetPosition, targetVelocity);
            } else {
              setPositionVelocityControl(
                  Rotations.of(profileState.position),
                  RotationsPerSecond.of(profileState.velocity));
            }
          }
        };

    command.addRequirements(this);

    return command;
  }

  /**
   * Zeroes the hood without the hall effect sensor by assuming the hood is at the minimum angle.
   *
   * @return A command that zeroes the hood
   */
  public Command zero() {
    return Commands.runOnce(
            () -> {
              hoodMotor.setPosition(ShooterHoodConstants.MIN_ANGLE);
              isZeroed = true;
            })
        .ignoringDisable(true);
  }
}
