package frc.robot.subsystems;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class AlgaeSubsystem extends SubsystemBase {
  private final SparkBase intake;

  public AlgaeSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/algae");

    try {
      ConfiguredMotor configuredMotor =
          new MotorParser(directory).withMotor("motor.json").configure();

      intake = configuredMotor.getSpark();
    } catch (IOException exception) {
      throw new RuntimeException("Exception configuring AlgaeSubsystem motor(s)", exception);
    }
  }

  /**
   * Run the algae intake at the supplied speed.
   *
   * @param speed The speed to run at.
   * @return The intake run command.
   */
  public Command runIntake(IntakeSpeed speed) {
    return runOnce(() -> intake.set(speed.getSpeed()));
  }

  public enum IntakeSpeed {
    REVERSE(-1),
    OFF(0),
    FULL(1);

    private double speed;

    private IntakeSpeed(double speed) {
      this.speed = speed;
    }

    public double getSpeed() {
      return speed;
    }
  }
}
