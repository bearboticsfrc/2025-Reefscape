package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.CoralSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;

public class ReefScoreCommand {
  public static Command get(
      ElevatorPosition position, ElevatorSubsystem elevator, CoralSubsystem coral) {
    return elevator
        .runElevatorTo(position)
        .andThen(Commands.waitUntil(elevator::isAtSetpoint))
        .andThen(coral.scoreCoral())
        .andThen(elevator.runElevatorTo(ElevatorPosition.HOME))
        .andThen(Commands.waitUntil(elevator::isAtSetpoint));
  }
}
