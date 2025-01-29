package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
  private final CoralSubsystem intake;

  public ManipulatorSubsystem() {
    this.intake = new CoralSubsystem();
  }
}
