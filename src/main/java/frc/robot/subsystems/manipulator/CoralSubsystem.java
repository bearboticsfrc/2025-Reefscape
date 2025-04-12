package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Seconds;

import bearlib.motor.MotorSpeed;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class CoralSubsystem extends SubsystemBase {
  private final int INTAKE_SENSOR_PORT = 2;
  private final int OUTAKE_SENSOR_PORT = 1;

  private final double CORAL_HONE_SPEED = 0.03;
  private final Time SCORE_WAIT_TIME = Seconds.of(0.1);

  @Logged(name = "Coral Intake Motor", importance = Importance.CRITICAL)
  private final SparkBase intake;

  @Logged(name = "Coral Outake Motor", importance = Importance.CRITICAL)
  private final SparkBase outake;

  @Logged(name = "Coral Outake Encoder", importance = Importance.CRITICAL)
  private final RelativeEncoder outakeEncoder;

  @Logged(name = "Intake Sensor", importance = Importance.CRITICAL)
  private final DigitalInput intakeSensor = new DigitalInput(INTAKE_SENSOR_PORT);

  @Logged(name = "Outake Sensor", importance = Importance.CRITICAL)
  private final DigitalInput outakeSensor = new DigitalInput(OUTAKE_SENSOR_PORT);

  /**
   * Constructs a CoralSubsystem.
   *
   * @throws RuntimeException If the motor configuration fails.
   */
  public CoralSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/coral");

    try {
      intake = new MotorParser(directory).withMotor("intake.json").configureAsync();
      outake =
          new MotorParser(directory)
              .withMotor("outake.json")
              .withPidf("pidf.json")
              .configureAsync();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure coral motor(s)!", exception);
    }

    outakeEncoder = outake.getEncoder();
  }

  /**
   * @return true if the coral is blocking the coral intake sensor.
   */
  @Logged(name = "Outake Has Coral", importance = Importance.CRITICAL)
  public boolean outakeHasCoral() {
    return !outakeSensor.get();
  }

  /**
   * @return true if a coral is blocking the coral intake sensor.
   */
  @Logged(name = "Intake Has Coral", importance = Importance.CRITICAL)
  public boolean intakeHasCoral() {
    return !intakeSensor.get();
  }

  /**
   * @return true if the coral is fully intaked.
   */
  @Logged(name = "Has Coral", importance = Importance.CRITICAL)
  public boolean hasCoral() {
    return !intakeHasCoral() && outakeHasCoral();
  }

  /**
   * @return true if the intake is active.
   */
  public boolean isIntakeActive() {
    return intake.get() != 0;
  }

  /**
   * Smart coral intake command.
   *
   * @return A {@link Command} intaking a coral.
   */
  public Command intakeCoral() {
    return Commands.either(coralPresentStrategy(), coralAbsentStrategy(), this::outakeHasCoral);
  }

  /**
   * Coral intake strategy when a coral is already present in the outake.
   *
   * @return A {@link Command} adjusting the coral.
   */
  private Command coralPresentStrategy() {
    return runOutake(MotorSpeed.TENTH)
        .alongWith(runIntake(MotorSpeed.TENTH))
        .andThen(Commands.waitUntil(() -> !intakeHasCoral()))
        .andThen(runOutake(MotorSpeed.REVERSE_TENTH))
        .andThen(honeCoral());
  }

  /**
   * Coral intake strategy when the coral intake is empty.
   *
   * @return A {@link Command} intaking a coral.
   */
  private Command coralAbsentStrategy() {
    return runOutake(MotorSpeed.TENTH)
        .alongWith(runIntake(MotorSpeed.TENTH))
        .andThen(Commands.waitUntil(this::intakeHasCoral))
        .andThen(honeCoral());
  }

  /**
   * Coral hone command. This ensures the coral is in the positioned properly.
   *
   * @return The {@link Command} honeing the coral.
   */
  private Command honeCoral() {
    return Commands.waitUntil(() -> !intakeHasCoral())
        .andThen(runOutake(MotorSpeed.REVERSE_TENTH).alongWith(runIntake(MotorSpeed.OFF)))
        .andThen(Commands.waitUntil(this::intakeHasCoral))
        .andThen(runOutake(CORAL_HONE_SPEED))
        .andThen(Commands.waitUntil(() -> !intakeHasCoral()))
        .andThen(stop());
  }

  /**
   * Coral score command
   *
   * @return A {@link Command} scoring a coral.
   */
  public Command scoreCoral() {
    return runOutake(MotorSpeed.FULL)
        .andThen(Commands.waitUntil(() -> !outakeHasCoral()))
        .andThen(Commands.waitTime(SCORE_WAIT_TIME))
        .andThen(runOutake(MotorSpeed.OFF));
  }

  /**
   * Coral score command
   *
   * @return A {@link Command} scoring a coral.
   */
  public Command teleopScoreCore() {
    return runOutake(MotorSpeed.QUARTER);
  }

  /**
   * Run the coral outake at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired outake motor speed.
   * @return A {@link Command} running the coral intake.
   */
  private Command runOutake(MotorSpeed speed) {
    return runOnce(() -> outake.set(speed.getSpeed()));
  }

  /**
   * Run the coral outake at the supplied speed.
   *
   * @param speed double describing the desired outake motor speed.
   * @return A {@link Command} running the coral intake.
   */
  private Command runOutake(double speed) {
    return Commands.runOnce(() -> outake.set(speed));
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
    return Commands.runOnce(this::stopOutake).alongWith(Commands.runOnce(intake::stopMotor));
  }

  public void stopOutake() {
    outake
        .getClosedLoopController()
        .setReference(outakeEncoder.getPosition(), ControlType.kPosition);
  }
}
