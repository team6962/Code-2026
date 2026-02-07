package com.team6962.lib.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/**
 * Represents a simulated linear mechanism. This code is based on WPILib's builtin {@link
 * ElevatorSim} but has been extended to allow changing characteristics at runtime and simulating a
 * diagonal elevator or an elevator with a constant force spring.
 */
public class LinearMechanismSim extends LinearSystemSim<N2, N1, N2> {
  // Gearbox for the linear mechanism.
  private DCMotor m_gearbox;

  // The min allowable height for the linear mechanism.
  private double m_minHeight;

  // The max allowable height for the linear mechanism.
  private double m_maxHeight;

  // The constant acceleration to simulate, such as gravity.
  private double m_constantAcceleration;

  /**
   * Creates a simulated linear mechanism.
   *
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param gearing The gearing of the linear mechanism (numbers greater than 1 represent
   *     reductions).
   * @param carriageMass The mass of the linear mechanism carriage.
   * @param drumRadius The radius of the drum that the linear mechanism spool is wrapped around.
   * @param minHeight The min allowable height of the linear mechanism.
   * @param maxHeight The max allowable height of the linear mechanism.
   * @param constantAcceleration The constant acceleration to simulate, such as gravity or a
   *     constant force spring. For a vertical elevator's gravity, this should be set to -9.81
   *     m/s^2. For no simulated constant acceleration, set this to 0.
   * @param startingHeight The starting height of the linear mechanism.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  public LinearMechanismSim(
      DCMotor gearbox,
      double gearing,
      Mass carriageMass,
      Distance drumRadius,
      Distance minHeight,
      Distance maxHeight,
      LinearAcceleration constantAcceleration,
      Distance startingHeight,
      double... measurementStdDevs) {
    this(
        LinearSystemId.createElevatorSystem(
            gearbox, carriageMass.in(Kilograms), drumRadius.in(Meters), gearing),
        gearbox,
        minHeight,
        maxHeight,
        constantAcceleration,
        startingHeight,
        measurementStdDevs);
  }

  /**
   * Creates a simulated linear mechanism.
   *
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param gearing The gearing of the linear mechanism (numbers greater than 1 represent
   *     reductions).
   * @param carriageMass The mass of the linear mechanism carriage.
   * @param drumRadius The radius of the drum that the linear mechanism spool is wrapped around.
   * @param minHeight The min allowable position of the linear mechanism.
   * @param maxHeight The max allowable position of the linear mechanism.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingHeight The starting position of the linear mechanism.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  public LinearMechanismSim(
      DCMotor gearbox,
      double gearing,
      Mass carriageMass,
      Distance drumRadius,
      Distance minHeight,
      Distance maxHeight,
      Angle angle,
      Distance startingHeight,
      double... measurementStdDevs) {
    this(
        LinearSystemId.createElevatorSystem(
            gearbox, carriageMass.in(Kilograms), drumRadius.in(Meters), gearing),
        gearbox,
        minHeight,
        maxHeight,
        MetersPerSecondPerSecond.of(-9.81).times(Math.sin(angle.in(Radians))),
        startingHeight,
        measurementStdDevs);
  }

  /**
   * Creates a simulated linear mechanism.
   *
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param minHeight The min allowable position of the linear mechanism.
   * @param maxHeight The max allowable position of the linear mechanism.
   * @param angle The angle of the linear mechanism from horizontal. For a vertical elevator, this
   *     should be 90 degrees. The angle is used to calculate the gravitational acceleration
   *     component along the mechanism's axis.
   * @param startingHeight The starting position of the linear mechanism.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  public LinearMechanismSim(
      double kV,
      double kA,
      DCMotor gearbox,
      Distance minHeight,
      Distance maxHeight,
      Angle angle,
      Distance startingHeight,
      double... measurementStdDevs) {
    this(
        LinearSystemId.identifyPositionSystem(kV, kA),
        gearbox,
        minHeight,
        maxHeight,
        MetersPerSecondPerSecond.of(-9.81).times(Math.sin(angle.in(Radians))),
        startingHeight,
        measurementStdDevs);
  }

  /**
   * Creates a simulated linear mechanism.
   *
   * @param plant The linear system that represents the linear mechanism. This system can be created
   *     with {@link edu.wpi.first.math.system.plant.LinearSystemId#createElevatorSystem(DCMotor,
   *     double, double, double)}.
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param minHeight The min allowable position of the linear mechanism.
   * @param maxHeight The max allowable position of the linear mechanism.
   * @param constantAcceleration The constant acceleration to simulate, such as gravity or a
   *     constant force spring. For a vertical elevator's gravity, this should be set to -9.81
   *     m/s^2. For no simulated constant acceleration, set this to 0.
   * @param startingHeight The starting position of the linear mechanism.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  @SuppressWarnings("this-escape")
  public LinearMechanismSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      Distance minHeight,
      Distance maxHeight,
      LinearAcceleration constantAcceleration,
      Distance startingHeight,
      double... measurementStdDevs) {
    super(plant, measurementStdDevs);

    m_gearbox = gearbox;
    m_minHeight = minHeight.in(Meters);
    m_maxHeight = maxHeight.in(Meters);
    m_constantAcceleration = constantAcceleration.in(MetersPerSecondPerSecond);

    setState(startingHeight.in(Meters), 0);
  }

  /**
   * Creates a simulated linear mechanism.
   *
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param minHeight The min allowable position of the linear mechanism.
   * @param maxHeight The max allowable position of the linear mechanism.
   * @param constantAcceleration The constant acceleration to simulate, such as gravity or a
   *     constant force spring. For a vertical elevator's gravity, this should be set to -9.81
   *     m/s^2. For no simulated constant acceleration, set this to 0.
   * @param startingHeight The starting position of the linear mechanism.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  public LinearMechanismSim(
      double kV,
      double kA,
      DCMotor gearbox,
      Distance minHeight,
      Distance maxHeight,
      LinearAcceleration constantAcceleration,
      Distance startingHeight,
      double... measurementStdDevs) {
    this(
        LinearSystemId.identifyPositionSystem(kV, kA),
        gearbox,
        minHeight,
        maxHeight,
        constantAcceleration,
        startingHeight,
        measurementStdDevs);
  }

  /**
   * Updates the system's characteristics.
   *
   * @param plant The new linear system representing the linear mechanism.
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param minHeight The min allowable position of the linear mechanism.
   * @param maxHeight The max allowable position of the linear mechanism.
   * @param constantAcceleration The constant acceleration to simulate, such as gravity or a
   *     constant force spring. For a vertical elevator's gravity, this should be set to -9.81
   *     m/s^2. For no simulated constant acceleration, set this to 0.
   */
  public void setCharacteristics(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      Distance minHeight,
      Distance maxHeight,
      LinearAcceleration constantAcceleration) {
    Matrix<N2, N2> newA = plant.getA();
    Matrix<N2, N1> newB = plant.getB();

    Matrix<N2, N2> a = m_plant.getA();
    Matrix<N2, N1> b = m_plant.getB();

    a.set(0, 0, newA.get(0, 0));
    a.set(0, 1, newA.get(0, 1));
    a.set(1, 0, newA.get(1, 0));
    a.set(1, 1, newA.get(1, 1));
    b.set(0, 0, newB.get(0, 0));
    b.set(1, 0, newB.get(1, 0));

    m_gearbox = gearbox;
    m_minHeight = minHeight.in(Meters);
    m_maxHeight = maxHeight.in(Meters);
    m_constantAcceleration = constantAcceleration.in(MetersPerSecondPerSecond);
  }

  /**
   * Updates the system's characteristics using physical parameters.
   *
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param gearing The gearing of the linear mechanism (numbers greater than 1 represent
   *     reductions).
   * @param carriageMass The mass of the linear mechanism carriage.
   * @param drumRadius The radius of the drum that the linear mechanism spool is wrapped around.
   * @param minHeight The min allowable height of the linear mechanism.
   * @param maxHeight The max allowable height of the linear mechanism.
   * @param constantAcceleration The constant acceleration to simulate, such as gravity or a
   *     constant force spring. For a vertical elevator's gravity, this should be set to -9.81
   *     m/s^2. For no simulated constant acceleration, set this to 0.
   */
  public void setCharacteristics(
      DCMotor gearbox,
      double gearing,
      Mass carriageMass,
      Distance drumRadius,
      Distance minHeight,
      Distance maxHeight,
      LinearAcceleration constantAcceleration) {
    setCharacteristics(
        LinearSystemId.createElevatorSystem(
            gearbox, carriageMass.in(Kilograms), drumRadius.in(Meters), gearing),
        gearbox,
        minHeight,
        maxHeight,
        constantAcceleration);
  }

  /**
   * Updates the system's characteristics using physical parameters with angle-based gravity.
   *
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param gearing The gearing of the linear mechanism (numbers greater than 1 represent
   *     reductions).
   * @param carriageMass The mass of the linear mechanism carriage.
   * @param drumRadius The radius of the drum that the linear mechanism spool is wrapped around.
   * @param minHeight The min allowable height of the linear mechanism.
   * @param maxHeight The max allowable height of the linear mechanism.
   * @param angle The angle of the linear mechanism from horizontal. For a vertical elevator, this
   *     should be 90 degrees. The angle is used to calculate the gravitational acceleration
   *     component along the mechanism's axis.
   */
  public void setCharacteristics(
      DCMotor gearbox,
      double gearing,
      Mass carriageMass,
      Distance drumRadius,
      Distance minHeight,
      Distance maxHeight,
      Angle angle) {
    setCharacteristics(
        LinearSystemId.createElevatorSystem(
            gearbox, carriageMass.in(Kilograms), drumRadius.in(Meters), gearing),
        gearbox,
        minHeight,
        maxHeight,
        MetersPerSecondPerSecond.of(-9.81).times(Math.sin(angle.in(Radians))));
  }

  /**
   * Updates the system's characteristics using system identification gains with constant
   * acceleration.
   *
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param minHeight The min allowable position of the linear mechanism.
   * @param maxHeight The max allowable position of the linear mechanism.
   * @param constantAcceleration The constant acceleration to simulate, such as gravity or a
   *     constant force spring. For a vertical elevator's gravity, this should be set to -9.81
   *     m/s^2. For no simulated constant acceleration, set this to 0.
   */
  public void setCharacteristics(
      double kV,
      double kA,
      DCMotor gearbox,
      Distance minHeight,
      Distance maxHeight,
      LinearAcceleration constantAcceleration) {
    setCharacteristics(
        LinearSystemId.identifyPositionSystem(kV, kA),
        gearbox,
        minHeight,
        maxHeight,
        constantAcceleration);
  }

  /**
   * Updates the system's characteristics using system identification gains with angle-based
   * gravity.
   *
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param gearbox The type of and number of motors in the linear mechanism gearbox.
   * @param minHeight The min allowable position of the linear mechanism.
   * @param maxHeight The max allowable position of the linear mechanism.
   * @param angle The angle of the linear mechanism from horizontal. For a vertical elevator, this
   *     should be 90 degrees. The angle is used to calculate the gravitational acceleration
   *     component along the mechanism's axis.
   */
  public void setCharacteristics(
      double kV, double kA, DCMotor gearbox, Distance minHeight, Distance maxHeight, Angle angle) {
    setCharacteristics(
        LinearSystemId.identifyPositionSystem(kV, kA),
        gearbox,
        minHeight,
        maxHeight,
        MetersPerSecondPerSecond.of(-9.81).times(Math.sin(angle.in(Radians))));
  }

  /**
   * Sets the linear mechanism's state. The new position will be limited between the minimum and
   * maximum allowed positions.
   *
   * @param positionMeters The new position in meters.
   * @param velocityMetersPerSecond New velocity in meters per second.
   */
  public final void setState(double positionMeters, double velocityMetersPerSecond) {
    setState(
        VecBuilder.fill(
            MathUtil.clamp(positionMeters, m_minHeight, m_maxHeight), velocityMetersPerSecond));
  }

  /**
   * Sets the linear mechanism's state. The new position will be limited between the minimum and
   * maximum allowed positions.
   *
   * @param position The new position.
   * @param velocity The new velocity.
   */
  public final void setState(Distance position, LinearVelocity velocity) {
    setState(position.in(Meters), velocity.in(MetersPerSecond));
  }

  /**
   * Returns whether the linear mechanism would hit the lower limit.
   *
   * @param positionMeters The linear mechanism position.
   * @return Whether the linear mechanism would hit the lower limit.
   */
  public boolean wouldHitLowerLimit(double positionMeters) {
    return positionMeters <= this.m_minHeight;
  }

  /**
   * Returns whether the linear mechanism would hit the lower limit.
   *
   * @param position The linear mechanism position.
   * @return Whether the linear mechanism would hit the lower limit.
   */
  public boolean wouldHitLowerLimit(Distance position) {
    return wouldHitLowerLimit(position.in(Meters));
  }

  /**
   * Returns whether the linear mechanism would hit the upper limit.
   *
   * @param positionMeters The linear mechanism position.
   * @return Whether the linear mechanism would hit the upper limit.
   */
  public boolean wouldHitUpperLimit(double positionMeters) {
    return positionMeters >= this.m_maxHeight;
  }

  /**
   * Returns whether the linear mechanism would hit the upper limit.
   *
   * @param position The linear mechanism position.
   * @return Whether the linear mechanism would hit the upper limit.
   */
  public boolean wouldHitUpperLimit(Distance position) {
    return wouldHitUpperLimit(position.in(Meters));
  }

  /**
   * Returns whether the linear mechanism has hit the lower limit.
   *
   * @return Whether the linear mechanism has hit the lower limit.
   */
  public boolean hasHitLowerLimit() {
    return wouldHitLowerLimit(getPositionMeters());
  }

  /**
   * Returns whether the linear mechanism has hit the upper limit.
   *
   * @return Whether the linear mechanism has hit the upper limit.
   */
  public boolean hasHitUpperLimit() {
    return wouldHitUpperLimit(getPositionMeters());
  }

  /**
   * Returns the position of the linear mechanism.
   *
   * @return The position of the linear mechanism.
   */
  public double getPositionMeters() {
    return getOutput(0);
  }

  /**
   * Returns the position of the linear mechanism.
   *
   * @return The position of the linear mechanism.
   */
  public Distance getPosition() {
    return Meters.of(getPositionMeters());
  }

  /**
   * Returns the velocity of the linear mechanism.
   *
   * @return The velocity of the linear mechanism.
   */
  public double getVelocityMetersPerSecond() {
    return getOutput(1);
  }

  /**
   * Returns the velocity of the linear mechanism.
   *
   * @return The velocity of the linear mechanism.
   */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(getVelocityMetersPerSecond());
  }

  /**
   * Returns the linear mechanism current draw.
   *
   * @return The linear mechanism current draw.
   */
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
    // spinning 10x faster than the output
    // v = r w, so w = v/r
    double kA = 1 / m_plant.getB().get(1, 0);
    double kV = -m_plant.getA().get(1, 1) * kA;
    double motorVelocityRadPerSec = m_x.get(1, 0) * kV * m_gearbox.KvRadPerSecPerVolt;
    var appliedVoltage = m_u.get(0, 0);
    return m_gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage)
        * Math.signum(appliedVoltage);
  }

  /**
   * Returns the linear mechanism current draw.
   *
   * @return The linear mechanism current draw.
   */
  public Current getCurrentDraw() {
    return Amps.of(getCurrentDrawAmps());
  }

  /**
   * Sets the input voltage for the linear mechanism.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
    clampInput(RobotController.getBatteryVoltage());
  }

  /**
   * Sets the input voltage for the linear mechanism.
   *
   * @param voltage The input voltage.
   */
  public void setInputVoltage(Voltage voltage) {
    setInputVoltage(voltage.in(Volts));
  }

  /**
   * Updates the state of the linear mechanism.
   *
   * @param currentXhat The current state estimate.
   * @param u The system inputs (voltage).
   * @param dtSeconds The time difference between controller updates.
   */
  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // Calculate updated x-hat from Runge-Kutta.
    var updatedXhat =
        NumericalIntegration.rkdp(
            (x, _u) -> {
              Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
              xdot = xdot.plus(VecBuilder.fill(0, m_constantAcceleration));
              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // We check for collisions after updating x-hat.
    if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_minHeight, 0);
    }
    if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_maxHeight, 0);
    }
    return updatedXhat;
  }
}
