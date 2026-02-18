package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    private final BeltFloor beltFloor;
    private final Kicker kicker;
    private final HopperSensors sensors;

    public Hopper() {
        this.beltFloor = new BeltFloor();
        this.kicker = new Kicker();
        this.sensors = new HopperSensors();
    }

    public Command load() {
        return Commands.parallel(
            beltFloor.feed(),
            kicker.feed()
        ).until(sensors::isKickerFull());
    }

    public Command feed() {
        Command FeedSystem = Commands.parallel(
            beltFloor.feed(),
            kicker.feed() 
        );
        return Commands.either(
            FeedSystem.withTimeout(0.25),
            FeedSystem.until(sensors::isKickerEmpty),
            sensors::isKickerEmpty
        );
    }

    public boolean isFull() {
        return sensors.isHopperFull();
    }

    public boolean isEmpty() {
        return sensors.isHopperEmpty() && sensors.isKickerEmpty(); 
    }

    public BeltFloor getBeltFloor() {
        return beltFloor;
    }

    public Kicker getKicker() {
        return kicker;
    }

    public HopperSensors getSensors() {
        return sensors;
    }
}