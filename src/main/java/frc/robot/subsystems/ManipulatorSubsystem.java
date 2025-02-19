package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
  private final CoralSubsystem coral;

  public ManipulatorSubsystem() {
    coral = new CoralSubsystem();
  }

  public Command intakeCoral() {
    return coral.intakeCoral();
  }

  public Command scoreCoral(){
    return coral.scoreCoral();
  }
}
