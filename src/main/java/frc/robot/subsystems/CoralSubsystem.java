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
  private final int INTAKE_SENSOR_PORT = 0;

  private final SparkBase intake;
  private final DigitalInput intakeSensor = new DigitalInput(INTAKE_SENSOR_PORT);

  /**
   * Constructs a CoralSubsystem.
   *
   * @throws RuntimeException If the motor configuration fails.
   */
  public CoralSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/coral");

    try {
      ConfiguredMotor configuredMotor =
          new MotorParser(directory).withMotor("motor.json").configure();

      intake = configuredMotor.getSpark();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure coral motor(s)!", exception);
    }
  }

  /**
   * @return true if the coral is blocking the coral intake sensor.
   */
  public boolean isCoralInIntake() {
    return intakeSensor.get();
  }

  /**
   * Smart coral intake command.
   *
   * @return A {@link Command} intaking the coral.
   */
  public Command intakeCoral() {
    return runIntake(MotorSpeed.FULL)
        .andThen(Commands.waitUntil(this::isCoralInIntake))
        .andThen(Commands.waitUntil(() -> !isCoralInIntake()))
        .andThen(runIntake(MotorSpeed.REVERSE_TENTH))
        .andThen(Commands.waitUntil(this::isCoralInIntake))
        .andThen(runIntake(MotorSpeed.OFF));
  }

  /**
   * Run the coral intake at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired intake motor speed.
   * @return A {@link Command} running the coral intake.
   */
  public Command runIntake(MotorSpeed speed) {
    return this.runOnce(() -> intake.set(speed.getSpeed()));
  }
}
