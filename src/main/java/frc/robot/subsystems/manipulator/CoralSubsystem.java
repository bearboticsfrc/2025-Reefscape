package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Seconds;

import bearlib.motor.MotorSpeed;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.epilogue.Logged;
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
  private final double RETRACT_THRESHOLD = -0.8;

  @Logged(name = "Coral Intake Motor")
  private final SparkBase intake;

  @Logged(name = "Coral Outake Motor")
  private final SparkBase outake;

  @Logged(name = "Coral Outake Encoder")
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
      intake = new MotorParser(directory).withMotor("intake.json").configureAsync();
      outake = new MotorParser(directory).withMotor("outake.json").configureAsync();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure coral motor(s)!", exception);
    }

    outakeEncoder = outake.getEncoder();
  }

  /**
   * @return true if the coral is blocking the coral intake sensor.
   */
  @Logged(name = "Has Coral")
  public boolean hasCoral() {
    return !intakeSensor.get();
  }

  /**
   * Smart coral intake command.
   *
   * @return A {@link Command} intaking the coral.
   */
  public Command intakeCoral() {
    return runOutake(MotorSpeed.REVERSE_TENTH)
        .alongWith(runIntake(MotorSpeed.TENTH))
        .andThen(Commands.waitUntil(this::hasCoral))
        .andThen(Commands.runOnce(() -> outakeEncoder.setPosition(0)))
        .andThen(Commands.waitUntil(() -> outakeEncoder.getPosition() <= RETRACT_THRESHOLD))
        .andThen(stop());
  }

  /**
   * Coral score command
   *
   * @return A {@link Command} scoring the coral.
   */
  public Command scoreCoral() {
    return runOutake(MotorSpeed.REVERSE_HALF)
        .andThen(Commands.waitTime(SCORING_TIME))
        .andThen(runOutake(MotorSpeed.OFF));
  }

  /**
   * Run the coral outake at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired outake motor speed.
   * @return A {@link Command} running the coral intake.
   */
  private Command runOutake(MotorSpeed speed) {
    return Commands.runOnce(() -> outake.set(speed.getSpeed()));
  }

  /**
   * Run the intake outake at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired intake motor speed.
   * @return A {@link Command} running the coral intake.
   */
  private Command runIntake(MotorSpeed speed) {
    return Commands.runOnce(() -> intake.set(speed.getSpeed()));
  }

  /**
   * @return A {@link Command} stopping both the intake and outake motors.
   */
  public Command stop() {
    return Commands.runOnce(outake::stopMotor).alongWith(Commands.runOnce(intake::stopMotor));
  }
}
