package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.CoralSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;

public class ReefScoreCommand {
  /**
   * Returns a command which scores coral on {@code position} position of the reef.
   *
   * <p>The order of operations is described below:
   *
   * <ol>
   *   <li>Run the elevator to {@code position}
   *   <li>Score the coral
   *   <li>Set the elevator to {@link ElevatorPosition#HOME}
   * </ol>
   *
   * <p>Note: While the command WILL wait for the elevator to reach {@code position}, it will NOT
   * wait for the elevator to home.
   *
   * @param position The elevator position to score at.
   * @param elevator A reference to the elevator subsystem.
   * @param coral A reference to the coral subsystem.
   * @return The command.
   */
  public static Command get(
      ElevatorPosition position, ElevatorSubsystem elevator, CoralSubsystem coral) {
    return elevator
        .runElevatorTo(position)
        .andThen(Commands.waitUntil(elevator::isAtSetpoint))
        .andThen(coral.scoreCoral())
        .andThen(elevator.runElevatorTo(ElevatorPosition.HOME));
  }
}
