package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.MotorSpeed;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class CoralSubsystem extends SubsystemBase {
  private final int INTAKE_SENSOR_PORT = 1;
  private final Time SCORING_TIME = Seconds.of(0.5);

  private final SparkBase intake;
  private final SparkBase outake;

  private final RelativeEncoder outakeEncoder;

  private final DigitalInput intakeSensor = new DigitalInput(INTAKE_SENSOR_PORT);

  /**
   * Constructs a CoralSubsystem.
   *
   * @throws RuntimeException If the motor configuration fails.
   */
  public CoralSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/coral");

    try {
      ConfiguredMotor configuredIntake =
          new MotorParser(directory).withMotor("intake.json").configure();
      ConfiguredMotor configuredOutake =
          new MotorParser(directory).withMotor("outake.json").configure();

      intake = configuredIntake.getSpark();
      outake = configuredOutake.getSpark();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure coral motor(s)!", exception);
    }

    outakeEncoder = outake.getEncoder();
  }

  /**
   * @return true if the coral is blocking the coral intake sensor.
   */
  public boolean isCoralInIntake() {
    return !intakeSensor.get();
  }

  /**
   * Smart coral intake command.
   *
   * @return A {@link Command} intaking the coral.
   */
  public Command intakeCoral() {
    return runIntake(MotorSpeed.FULL)
        .alongWith(runOutake(MotorSpeed.REVERSE_QUARTER))
        .andThen(Commands.waitUntil(this::isCoralInIntake))
        .andThen(Commands.runOnce(() -> outakeEncoder.setPosition(0)))
        .andThen(runIntake(MotorSpeed.QUARTER).alongWith(runOutake(MotorSpeed.REVERSE_TENTH)))
        .andThen(Commands.waitUntil(() -> outakeEncoder.getPosition() <= -1))
        .andThen(stopIntake());
  }

  public Command scoreCoral() {
    return runOutake(MotorSpeed.REVERSE_FULL)
        .andThen(Commands.waitTime(SCORING_TIME))
        .andThen(runOutake(MotorSpeed.OFF));
  }

  /**
   * Run the coral intake at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired intake motor speed.
   * @return A {@link Command} running the coral intake.
   */
  public Command runIntake(MotorSpeed speed) {
    return Commands.runOnce(() -> intake.set(speed.getSpeed()));
  }

  /**
   * Run the coral outake at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired intake motor speed.
   * @return A {@link Command} running the coral intake.
   */
  public Command runOutake(MotorSpeed speed) {
    return Commands.runOnce(() -> outake.set(speed.getSpeed()));
  }

  public Command stopIntake() {
    return Commands.parallel(
        Commands.runOnce(intake::stopMotor), Commands.runOnce(outake::stopMotor));
  }
}
