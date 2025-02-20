package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.MotorSpeed;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class AlgaeSubsystem extends SubsystemBase {
  private final int ALGAE_SENSOR_PORT = 0;
  private final Time SCORING_TIME = Seconds.of(0.5);

  private final SparkBase motor;

  private final DigitalInput algaeSensor = new DigitalInput(ALGAE_SENSOR_PORT);

  /**
   * Constructs a AlgaeSubsystem.
   *
   * @throws RuntimeException If the motor configuration fails.
   */
  public AlgaeSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/algae");

    try {
      ConfiguredMotor configuredMotor =
          new MotorParser(directory).withMotor("motor.json").configure();

      motor = configuredMotor.getSpark();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure algae motor!", exception);
    }

    ShuffleboardTab sensors = Shuffleboard.getTab("Algae Sensors");

    sensors.addDouble("Output Current", motor::getOutputCurrent);
    sensors.addDouble("Applied Output", motor::getAppliedOutput);
    sensors.addDouble("Velocity", () -> motor.getEncoder().getVelocity());
  }

  /**
   * @return true if the algae is blocking the sensor.
   */
  public boolean hasAlgae() {
    return motor.getEncoder().getVelocity() < 0.1;
    // return !algaeSensor.get();
  }

  /**
   * Smart algae intake command.
   *
   * @return A {@link Command} intaking the algae.
   */
  public Command intakeAlgae() {
    return runMotor(MotorSpeed.QUARTER)
        .andThen(Commands.print("running at quarter"))
        .andThen(Commands.waitUntil(this::hasAlgae))
        .andThen(Commands.print("running at tenth"))
        .andThen(runMotor(MotorSpeed.TENTH));
  }

  public Command intakeAlgaeOld() {
    return runMotor(MotorSpeed.QUARTER)
        .andThen(Commands.print("running at quarter"))
        .andThen(Commands.waitUntil(this::hasAlgae))
        .andThen(Commands.print("running at tenth"))
        .andThen(runMotor(MotorSpeed.TENTH).finallyDo(() -> motor.stopMotor()));
  }

  public Command stopIntakeAlgae() {
    return stopMotor();
  }

  public Command scoreAlgae() {
    return runMotor(MotorSpeed.REVERSE_FULL)
        .andThen(Commands.waitTime(SCORING_TIME))
        .andThen(stopMotor());
  }

  /**
   * Run the motor at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired intake motor speed.
   * @return A {@link Command} running the algae intake.
   */
  public Command runMotor(MotorSpeed speed) {
    return Commands.runOnce(() -> motor.set(speed.getSpeed()));
  }

  public Command stopMotor() {
    return Commands.runOnce(motor::stopMotor);
  }
}
