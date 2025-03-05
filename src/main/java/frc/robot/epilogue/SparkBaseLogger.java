package frc.robot.epilogue;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkBase.class)
public class SparkBaseLogger extends ClassSpecificLogger<SparkBase> {
  public SparkBaseLogger() {
    super(SparkBase.class);
  }

  @Override
  public void update(EpilogueBackend backend, SparkBase spark) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("Forward Limit Pressed?", spark.getForwardLimitSwitch().isPressed());
      backend.log("Reversed Limit Pressed?", spark.getReverseLimitSwitch().isPressed());
      backend.log("Bus Voltage (V)", spark.getBusVoltage());
      backend.log("Applied Output (Duty Cycle)", spark.getAppliedOutput());
    }
    
    backend.log("Requested Speed (Duty Cycle)", spark.get());
    backend.log("Temperature (C)", spark.getMotorTemperature());
    backend.log("Output Current (A)", spark.getOutputCurrent());
  }
}
