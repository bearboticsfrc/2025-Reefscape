package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;

public class ManipulatorSubsystem extends SubsystemBase {
  private final CoralSubsystem coral = new CoralSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  public Command runElevatorTo(ElevatorPosition position) {
    return elevator.runElevatorTo(position);
  }

  public Command intakeCoral() {
    return coral.intakeCoral();
  }

  public Command scoreCoral() {
    return coral.scoreCoral();
  }
}
