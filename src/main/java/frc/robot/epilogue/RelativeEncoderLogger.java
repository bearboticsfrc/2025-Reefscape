package frc.robot.epilogue;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(RelativeEncoder.class)
public class RelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {
  public RelativeEncoderLogger() {
    super(RelativeEncoder.class);
  }

  @Override
  public void update(EpilogueBackend backend, RelativeEncoder encoder) {
    backend.log("Position (Rotations)", encoder.getPosition());
    backend.log("Velocity (RPM)", encoder.getVelocity());
  }
}
