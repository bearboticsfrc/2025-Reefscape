package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Seconds;

import bearlib.motor.MotorSpeed;
import bearlib.motor.deserializer.MotorParser;
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

public class AlgaeSubsystem extends SubsystemBase {
  private final int ALGAE_SENSOR_PORT = 0;
  private final Time SCORING_TIME = Seconds.of(0.5);
  private final double IDLE_INTAKE_SPEED = 0.1;

  @Logged(name = "Algae Motor")
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
      motor = new MotorParser(directory).withMotor("motor.json").configureAsync();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure algae motor!", exception);
    }
  }

  @Override
  public void periodic() {
    if (hasAlgae() && motor.get() == 0) {
      motor.set(IDLE_INTAKE_SPEED);
    }
  }

  /**
   * @return true if the algae is blocking the sensor.
   */
  @Logged(name = "Has Algae")
  public boolean hasAlgae() {
    return !algaeSensor.get();
  }

  /**
   * Smart algae intake command.
   *
   * @return A {@link Command} intaking the algae.
   */
  public Command intakeAlgae() {
    return run(MotorSpeed.QUARTER)
        .andThen(Commands.waitUntil(this::hasAlgae))
        .andThen(run(MotorSpeed.TENTH));
  }

  public Command scoreAlgae() {
    return run(MotorSpeed.REVERSE_FULL)
        .andThen(Commands.waitTime(SCORING_TIME))
        .andThen(stopMotor());
  }

  /**
   * Run the motor at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired intake motor speed.
   * @return A {@link Command} running the algae intake.
   */
  public Command run(MotorSpeed speed) {
    return Commands.runOnce(() -> motor.set(speed.getSpeed()));
  }

  public Command stopMotor() {
    return Commands.runOnce(motor::stopMotor);
  }
}
