package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import bearlib.motor.MotorSpeed;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class ClimberSubsystem extends SubsystemBase {
  private final int SERVO_CHANNEL = -1;

  private final Angle DEPLOY_ANGLE = Degrees.of(0);

  @Logged(name = "Winch Motor")
  private final SparkBase winch;

  @Logged(name = "Servo")
  private final Servo servo = new Servo(SERVO_CHANNEL);

  /**
   * Constructs a CoralSubsystem.
   *
   * @throws RuntimeException If the motor configuration fails.
   */
  public ClimberSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/climber");

    try {
      winch =
          new MotorParser(directory).withMotor("motor.json").withPidf("pidf.json").configureAsync();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure climber motor(s)!", exception);
    }
  }

  public Command deploy() {
    return Commands.runOnce(() -> servo.setAngle(DEPLOY_ANGLE.in(Degrees)))
        .andThen(runClimberTo(ClimberPosition.DEPLOY));
  }

  /**
   * Run the coral outake at the supplied speed.
   *
   * @param speed {@link MotorSpeed} describing the desired intake motor speed.
   * @return A {@link Command} running the coral intake.
   */
  private Command runClimberTo(ClimberPosition position) {
    SparkClosedLoopController controller = winch.getClosedLoopController();

    return Commands.runOnce(
        () -> controller.setReference(position.getPosition(), ControlType.kPosition));
  }

  /**
   * @return A {@link Command} stopping the climber motor.
   */
  public Command stop() {
    return Commands.runOnce(winch::stopMotor);
  }

  public enum ClimberPosition {
    DEPLOY(-1);

    private double position;

    private ClimberPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
