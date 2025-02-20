package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
  private final AlgaeSubsystem algae;
  private final CoralSubsystem coral;

  public ManipulatorSubsystem() {
    algae = new AlgaeSubsystem();
    coral = new CoralSubsystem();
  }

  public Command intakeCoral() {
    return coral.intakeCoral();
  }

  public Command scoreCoral() {
    return coral.scoreCoral();
  }

  public Command intakeAlgae() {
    return algae.intakeAlgae();
  }

  public Command stopIntakeAlgae() {
    return algae.stopIntakeAlgae();
  }

  public Command scoreAlgae() {
    return algae.scoreAlgae();
  }
}
