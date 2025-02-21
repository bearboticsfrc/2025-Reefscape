package frc.robot.epilogue;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

@CustomLoggerFor(State.class)
public class TrapezoidProfileStateLogger extends ClassSpecificLogger<State> {
  public TrapezoidProfileStateLogger() {
    super(State.class);
  }

  @Override
  public void update(EpilogueBackend backend, State state) {
    backend.log("Position", state.position);
    backend.log("Velocity", state.velocity);
  }
}
