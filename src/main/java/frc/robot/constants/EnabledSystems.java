package frc.robot.constants;

public class EnabledSystems {
  public boolean turret = true;
  public boolean shooterHood = true;
  public boolean shooterRollers = true;
  public boolean intakeExtension = true;
  public boolean intakeRollers = true;
  public boolean kicker = true;
  public boolean climb = true;
  public boolean drivetrain = true;
  public boolean beltFloor = true;
  public boolean hopperSensors = true;

  public EnabledSystems() {}

  public EnabledSystems withTurret(boolean enabled) {
    this.turret = enabled;
    return this;
  }

  public EnabledSystems withShooterHood(boolean enabled) {
    this.shooterHood = enabled;
    return this;
  }

  public EnabledSystems withShooterRollers(boolean enabled) {
    this.shooterRollers = enabled;
    return this;
  }

  public EnabledSystems withIntakeExtension(boolean enabled) {
    this.intakeExtension = enabled;
    return this;
  }

  public EnabledSystems withIntakeRollers(boolean enabled) {
    this.intakeRollers = enabled;
    return this;
  }

  public EnabledSystems withKicker(boolean enabled) {
    this.kicker = enabled;
    return this;
  }

  public EnabledSystems withClimb(boolean enabled) {
    this.climb = enabled;
    return this;
  }

  public EnabledSystems withDrivetrain(boolean enabled) {
    this.drivetrain = enabled;
    return this;
  }

  public EnabledSystems withBeltFloor(boolean enabled) {
    this.beltFloor = enabled;
    return this;
  }

  public EnabledSystems withHopperSensors(boolean enabled) {
    this.hopperSensors = enabled;
    return this;
  }

  public EnabledSystems withAll(boolean enabled) {
    return this.withDrivetrain(enabled).withSubsystems(enabled);
  }

  public EnabledSystems withSubsystems(boolean enabled) {
    return this.withTurret(enabled)
        .withShooterHood(enabled)
        .withShooterRollers(enabled)
        .withIntakeExtension(enabled)
        .withIntakeRollers(enabled)
        .withKicker(enabled)
        .withClimb(enabled)
        .withBeltFloor(enabled)
        .withHopperSensors(enabled);
  }

  public EnabledSystems withShooter(boolean enabled) {
    return this.withShooterHood(enabled).withShooterRollers(enabled).withTurret(enabled);
  }

  public EnabledSystems withHopper(boolean enabled) {
    return this.withKicker(enabled).withBeltFloor(enabled).withHopperSensors(enabled);
  }

  public EnabledSystems withIntake(boolean enabled) {
    return this.withIntakeRollers(enabled).withIntakeExtension(enabled);
  }

  public EnabledSystems and(EnabledSystems other) {
    return new EnabledSystems()
        .withTurret(this.turret && other.turret)
        .withShooterHood(this.shooterHood && other.shooterHood)
        .withShooterRollers(this.shooterRollers && other.shooterRollers)
        .withIntakeExtension(this.intakeExtension && other.intakeExtension)
        .withIntakeRollers(this.intakeRollers && other.intakeRollers)
        .withKicker(this.kicker && other.kicker)
        .withClimb(this.climb && other.climb)
        .withDrivetrain(this.drivetrain && other.drivetrain)
        .withBeltFloor(this.beltFloor && other.beltFloor)
        .withHopperSensors(this.hopperSensors && other.hopperSensors);
  }
}
