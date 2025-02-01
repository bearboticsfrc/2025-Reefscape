package frc.robot.commands;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class LinearControlEffLogCmd extends Command {
  SparkBase[] motors;
  DoubleSupplier[] powerSuppliers;
  private int commandCycles;
  private double powerSum = 0;
  private Timer timer;

  /**
   * @param motors each motor used in a linear control system logs the data from a linear control
   *     system. Includes: Work done by input control matrix, and avg power pulled. Two Dimensional
   *     Control model, the control input is the output of the control models on the motors
   *     themselves. For this type of system first order control logging should be done at a
   *     discretized lower level implementation. If you were to monitor this as a full system:
   *     Dimension one is dimension two, and dimension two is dimension three. Dimension one would
   *     be control input function implemented directly in the software. Linear control system as
   *     the control input matrix (which may be non linear). results in a linear differential
   *     dependency matrix. This class is designed as a wrapper for easy logging. This data should
   *     be used to optimize control input models via gradient recursion. Any lower level control
   *     system analysis should be done through more precise logging.
   */
  public LinearControlEffLogCmd(SparkBase[] motors) {
    this.motors = motors;
    this.powerSuppliers = new DoubleSupplier[motors.length];
    commandCycles = 0;
  }

  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();

    int i = 0;
    for (SparkBase motor : motors) {
      powerSuppliers[i] = () -> motor.getOutputCurrent() * motor.getBusVoltage();
      i++;
    }
  }

  @Override
  public void execute() {
    for (DoubleSupplier powerSupplier : powerSuppliers) {
      powerSum += powerSupplier.getAsDouble();
    }
    commandCycles++;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    double avgPower = powerSum / commandCycles;
    double work = avgPower * timer.get();
    DataLogManager.log("Work done by input control matrix: " + work);
    DataLogManager.log("Avg power pulled: " + avgPower);
  }
}
