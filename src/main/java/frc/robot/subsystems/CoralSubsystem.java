package frc.robot.subsystems;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.MotorSpeed;
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
    File directory = new File(Filesystem.getDeployDirectory(), "motors/coral");
    try {
      ConfiguredMotor configuredMotor =
          new MotorParser(directory).withMotor("motor.json").configure();

      this.intake = configuredMotor.getSpark();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure coral motor!", exception);
    }
  }

  /**
   * @return true if the coral is blocking the coral intake sensor Digital/Binary NPM banner
   *     proximity sensor
   */
  public boolean isCoralIn() {
    return intakeSensor.get();
  }

  /**
   * runs intake fast then backs it up slow into the robot
   *
   * @return Coral Grab Command
   */
  public Command getCoralGrabCommand() {
    return getCoralIntakeRunCommand(MotorSpeed.FULL)
        .andThen(Commands.waitUntil(() -> isCoralIn()))
        .andThen(Commands.waitUntil(() -> !isCoralIn()))
        .andThen(getCoralIntakeRunCommand(MotorSpeed.REVERSE_TENTH))
        .andThen(Commands.waitUntil(() -> isCoralIn()))
        .andThen(getCoralIntakeRunCommand(MotorSpeed.OFF));
  }

  /**
   * @param speed enum of discrete set of doubles in dom [-1, 1] Sets the speed of the coral intake
   *     motor the abs of the double is the speed coefficient out of 1 negative is reverse
   * @return Coral Intake Run Command
   */
  public Command getCoralIntakeRunCommand(MotorSpeed speed) {
    return this.runOnce(() -> this.intake.set(speed.getSpeed()));
  }
}
