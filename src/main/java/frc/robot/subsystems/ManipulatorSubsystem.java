package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
  private final CoralSubsystem coral;

  public ManipulatorSubsystem() {
    this.coral = new CoralSubsystem();
  }
}
