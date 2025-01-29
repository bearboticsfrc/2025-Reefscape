package frc.robot.subsystems;

import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class CoralSubsystem extends SubsystemBase {
  private MotorParser intakeParser;
  private SparkBase intake;
  private DigitalInput intakeSensor = new DigitalInput(0);

  /**
   * Constructs a CoralSubsystem. Coral Subsystem fully develops the low level architecture for the
   * Coral Intake. Coral Subsystem also provides functionality for full high level control over the
   * Intake
   */
  public CoralSubsystem() {
    configureMotors();
  }

  /**
   * pulls the Json motor files and passes them into the motor wrapper/parser
   *
   * @throws RuntimeException if the motor configuration fails
   */
  private void configureMotors() {
    File directory = new File(Filesystem.getDeployDirectory(), "Elevator");
    try {
      intakeParser = new MotorParser(directory).withMotor("intakeMotor.json");
      this.intake = this.intakeParser.configure().getSpark();
    } catch (IOException e) {
      throw new RuntimeException("Failed to configure motors: " + e.getCause().toString());
    }
  }

  /**
   * @return true if the coral is blocking the coral intake sensor Digital/Binary NPM banner
   *     proximity sensor
   */
  public boolean isCoralIn() {
    return this.intakeSensor.get();
  }

  /**
   * runs intake fast then backs it up slow into the robot
   *
   * @return Coral Grab Command
   */
  public Command getCoralGrabCommand() {
    return getCoralIntakeRunCommand(CoralIntakeSpeed.FULL)
        .andThen(Commands.waitUntil(this::isCoralIn))
        .andThen(Commands.waitUntil(() -> !this.isCoralIn()))
        .andThen(getCoralIntakeRunCommand(CoralIntakeSpeed.TENTH_R))
        .andThen(Commands.waitUntil(this::isCoralIn))
        .andThen(getCoralIntakeRunCommand(CoralIntakeSpeed.OFF));
  }

  /**
   * @param speed enum of discrete set of doubles in dom [-1, 1] Sets the speed of the coral intake
   *     motor the abs of the double is the speed coefficient out of 1 negative is reverse
   * @return Coral Intake Run Command
   */
  public Command getCoralIntakeRunCommand(CoralIntakeSpeed speed) {
    return this.runOnce(() -> this.intake.set(speed.getSpeed()));
  }

  /** Enum representing different intake speeds. */
  public enum CoralIntakeSpeed {
    R(-1),
    TENTH_R(-0.1),
    OFF(0),
    TENTH(0.1),
    QUARTER(0.25),
    HALF(0.5),
    FULL(1);

    private final double speed;

    /**
     * Constructor for IntakeSpeed.
     *
     * @param speed The speed value associated with the intake speed.
     */
    private CoralIntakeSpeed(double speed) {
      this.speed = speed;
    }

    /**
     * Get the speed value associated with the intake speed.
     *
     * @return The speed value.
     */
    public double getSpeed() {
      return speed;
    }
  }
}
