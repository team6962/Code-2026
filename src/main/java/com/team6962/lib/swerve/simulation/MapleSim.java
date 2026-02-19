package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Voltage;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * Wrapper for MapleSim physics simulation that manages the 2026 game arena and swerve drive
 * simulation.
 *
 * <p>This class initializes and manages the MapleSim physics engine, which provides realistic
 * simulation of swerve drive mechanics, game piece interactions, and field collisions. It
 * configures the simulation timing, arena setup, and drivetrain characteristics based on the
 * robot's configuration.
 *
 * <p>The simulation includes:
 *
 * <ul>
 *   <li>Field elements and game pieces for the 2026 season
 *   <li>Physics-based swerve drive motion with realistic friction and dynamics
 *   <li>Optional collision detection with field ramps
 *   <li>Performance optimization modes
 * </ul>
 */
public class MapleSim {
  /** The 2026 season-specific arena instance containing field elements and game pieces. */
  private Arena2026Rebuilt arena;

  /** The swerve drive simulation instance that handles drivetrain physics. */
  private SwerveDriveSimulation swerveSim;

  /**
   * Creates and initializes the MapleSim physics simulation.
   *
   * <p>This constructor:
   *
   * <ol>
   *   <li>Configures simulation timing based on the robot's control loop frequency
   *   <li>Creates the 2026 arena with optional ramp collision detection
   *   <li>Sets up the swerve drive simulation with motor characteristics and geometry
   *   <li>Registers the drivetrain with the arena for physics interactions
   *   <li>Initializes the field for autonomous mode
   * </ol>
   *
   * @param drivetrainConstants the drivetrain configuration containing motor specs, dimensions, and
   *     simulation settings
   */
  public MapleSim(DrivetrainConstants drivetrainConstants) {
    // Configure the simulation timing to match the robot's control loop period and
    // set the number of physics sub-steps per period for accuracy
    SimulatedArena.overrideSimulationTimings(
        drivetrainConstants.Timing.ControlLoopFrequency.asPeriod(),
        drivetrainConstants.Simulation.SimulationSubTicksPerPeriod);

    // Create the 2026 season arena with optional ramp collision detection.
    // When enabled, the robot can collide with and drive over field ramps.
    arena = new Arena2026Rebuilt(drivetrainConstants.Simulation.EnableRampCollider);

    // Enable efficiency mode to reduce CPU usage by simplifying certain physics calculations.
    // This can improve performance on slower computers at the cost of some accuracy.
    arena.setEfficiencyMode(drivetrainConstants.Simulation.EnableEfficiencyMode);

    // Register this arena as the global simulation instance so other components can access it
    SimulatedArena.overrideInstance(arena);

    List<Supplier<SwerveModuleSimulation>> moduleSuppliers = new LinkedList<>();

    for (int i = 0; i < drivetrainConstants.SwerveModules.length; i++) {
      final int moduleIndex = i;

      moduleSuppliers.add(
          () ->
              new SwerveModuleSimulation(
                  new SwerveModuleSimulationConfig(
                      drivetrainConstants.DriveMotor.SimulatedMotor,
                      drivetrainConstants.SteerMotor.SimulatedMotor,
                      drivetrainConstants.DriveMotor.GearReduction,
                      drivetrainConstants.getSteerGearReduction(moduleIndex),
                      getKS(
                          drivetrainConstants.getDriveMotorConfig(moduleIndex),
                          drivetrainConstants.DriveMotor.PositionSlot),
                      getKS(
                          drivetrainConstants.getSteerMotorConfig(moduleIndex),
                          drivetrainConstants.SteerMotor.PositionSlot),
                      drivetrainConstants.getWheelRadius(moduleIndex),
                      drivetrainConstants.getSteerMomentOfInertia(moduleIndex),
                      drivetrainConstants.Structure.WheelCOF)));
    }

    @SuppressWarnings("unchecked")
    Supplier<SwerveModuleSimulation>[] moduleSuppliersArray =
        moduleSuppliers.toArray(new Supplier[0]);

    // Create the swerve drive simulation with the robot's specific configuration
    swerveSim =
        new SwerveDriveSimulation(
            DriveTrainSimulationConfig.Default()
                // Configure the gyroscope (Pigeon2) for heading measurements
                .withGyro(COTS.ofPigeon2())
                // Configure each swerve module with the appropriate motor types, gear reductions,
                // feedforward values, and physical properties
                .withSwerveModules(moduleSuppliersArray)
                // Set the drivetrain dimensions (distance between left/right wheels and front/back
                // wheels)
                .withTrackLengthTrackWidth(
                    drivetrainConstants.Structure.TrackWidth,
                    drivetrainConstants.Structure.WheelBase)
                // Set the robot bumper dimensions for collision detection
                .withBumperSize(
                    drivetrainConstants.Structure.OuterWidth,
                    drivetrainConstants.Structure.OuterLength),
            // Set the initial pose of the robot on the field
            drivetrainConstants.Simulation.InitialPose);

    // Register the swerve drive with the arena so it participates in physics simulation
    arena.addDriveTrainSimulation(swerveSim);

    // Reset the field to its autonomous starting configuration with game pieces in place
    arena.resetFieldForAuto();
  }

  /**
   * Gets the swerve drive simulation instance for direct access to drivetrain simulation data.
   *
   * @return the {@link SwerveDriveSimulation} instance managing swerve module physics
   */
  public SwerveDriveSimulation getSwerveSim() {
    return swerveSim;
  }

  /**
   * Gets the arena simulation instance for direct access to field elements, game pieces, and arena
   * interactions.
   *
   * @return the {@link Arena2026Rebuilt} instance representing the 2026 season arena simulation
   */
  public Arena2026Rebuilt getArena() {
    return arena;
  }

  /**
   * Gets the robot pose as computed by the MapleSim physics simulation.
   *
   * @return the current estimated position and orientation of the robot on the field based on the
   *     physics simulation
   */
  public Pose2d getRobotPose() {
    return swerveSim.getSimulatedDriveTrainPose();
  }

  /**
   * Gets the poses of all fuel game pieces currently in the arena simulation.
   *
   * @return an array of {@link Pose3d} representing the positions and orientations of all fuel game
   *     pieces currently in the arena simulation
   */
  public Pose3d[] getFuelPositions() {
    return arena.getGamePiecesArrayByType("Fuel");
  }

  /**
   * Updates the physics simulation for one time step.
   *
   * <p>This method advances the arena simulation, which updates all physics objects including the
   * drivetrain, game pieces, and field interactions. It also logs the current positions of all fuel
   * game pieces for debugging and visualization.
   *
   * <p>This should be called periodically during simulation mode.
   *
   * @param deltaTime the time elapsed since the last update (currently unused as the arena uses the
   *     timing configured in the constructor)
   */
  public void update(double deltaTime) {
    // Run the arena's periodic update to advance all physics simulations
    arena.simulationPeriodic();

    // Query the positions of all fuel game pieces in the arena
    Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");

    // Log fuel positions for visualization in AdvantageScope or other logging tools
    DogLog.log("Drivetrain/Simulation/FuelPositions", fuelPoses);
  }

  /**
   * Helper method to extract the static feedforward voltage (kS) from a TalonFX motor configuration
   * for use in the physics simulation.
   *
   * @param config the TalonFX motor configuration containing the feedforward values
   * @param slotIndex the index of the PID slot to extract the kS value from (typically 0 for
   *     primary slot)
   * @return the static feedforward voltage (kS) as a {@link Voltage} object
   */
  private Voltage getKS(TalonFXConfiguration config, int slotIndex) {
    switch (slotIndex) {
      case 0:
        return Volts.of(config.Slot0.kS);
      case 1:
        return Volts.of(config.Slot1.kS);
      case 2:
        return Volts.of(config.Slot2.kS);
      default:
        throw new IllegalArgumentException("Invalid slot index: " + slotIndex);
    }
  }
}
