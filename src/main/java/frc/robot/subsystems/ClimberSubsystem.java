package frc.robot.subsystems;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkBase winch;

  public ClimberSubsystem() {
    ConfiguredMotor configuredMotor;

    try {
      File directory = new File(Filesystem.getDeployDirectory(), "climber");

      configuredMotor = new MotorParser(directory).withMotor("motor.json").configure();
    } catch (IOException exception) {
      throw new RuntimeException("Exception configuring ClimberSubsytem: " + exception);
    }

    this.winch = configuredMotor.getSpark();
  }

  /**
   * Run the winch at the supplied speed.
   *
   * @param speed The speed to run the winch at.
   */
  public void runWinch(WinchSpeed speed) {
    winch.set(speed.getSpeed());
  }

  public enum WinchSpeed {
    OFF(0),
    FULL(1);

    private double speed;

    private WinchSpeed(double speed) {
      this.speed = speed;
    }

    public double getSpeed() {
      return speed;
    }
  }
}
