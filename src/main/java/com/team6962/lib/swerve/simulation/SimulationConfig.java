package com.team6962.lib.swerve.simulation;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulationConfig {
    public LinearSystem<N2, N1, N2> driveSimulationPlant;
    public DCMotor driveSimulationGearbox;

    public LinearSystem<N2, N1, N2> steerSimulationPlant;
    public DCMotor steerSimulationGearbox;
}
