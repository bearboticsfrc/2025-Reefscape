package frc.robot.subsystems;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.MotorSpeed;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import edu.wpi.first.wpilibj.Servo;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkBase winch;
  private final int SERVO_PWM_PORT = 1;
  private Servo servo = new Servo(SERVO_PWM_PORT);
    
  public void release() {
      servo.set(1.0);
  }


  public ClimberSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/climber");

    try {
      ConfiguredMotor configuredMotor =
          new MotorParser(directory).withMotor("motor.json").configure();

      winch = configuredMotor.getSpark();
    } catch (IOException exception) {
      throw new RuntimeException("Exception configuring ClimberSubsytem motor(s)", exception);
    }
  }

  /**
   * Returns a command to run the winch.
   *
   * @param speed The speed to run the winch at.
   * @return The command to run the winch.
   */
  public Command runWinch(MotorSpeed speed) {
    return runOnce(() -> winch.set(speed.getSpeed()));
  }
}
