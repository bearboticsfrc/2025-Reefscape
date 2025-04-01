package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulator.CoralSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;

/**
 * A command group that scores a coral piece onto the reef at a specified elevator position.
 *
 * <p>This command orchestrates the elevator and coral subsystems to perform the scoring sequence.
 */
public class ReefScoreCommand extends SequentialCommandGroup {

  /**
   * Creates a command sequence to score a coral piece onto the reef at the specified height.
   *
   * <p>The sequence performs the following actions:
   *
   * <ol>
   *   <li>Waits until the {@link CoralSubsystem} confirms it is holding a coral piece ready for
   *       scoring (via {@code coral.outakeHasCoral()}).
   *   <li>Commands the {@link ElevatorSubsystem} to move to the target scoring {@code position}.
   *   <li>Waits until the {@link ElevatorSubsystem} reports being at the desired setpoint (via
   *       {@code elevator.isAtSetpoint()}).
   *   <li>Commands the {@link CoralSubsystem} to execute its scoring action (via {@code
   *       coral.scoreCoral()}).
   *   <li>Commands the {@link ElevatorSubsystem} to start moving to the {@link
   *       ElevatorPosition#HOME} position.
   * </ol>
   *
   * <p><b>Important Note:</b> While this command sequence waits for the elevator to reach the
   * target scoring {@code position} before scoring, it does <i>not</i> wait for the elevator to
   * finish returning to the {@link ElevatorPosition#HOME} position. The command group completes
   * once the homing movement is initiated.
   *
   * @param position The target {@link ElevatorPosition} for scoring.
   * @param elevator The {@link ElevatorSubsystem} instance.
   * @param coral The {@link CoralSubsystem} instance.
   */
  public ReefScoreCommand(
      final ElevatorPosition position,
      final ElevatorSubsystem elevator,
      final CoralSubsystem coral) {

    addCommands(
        Commands.waitUntil(coral::outakeHasCoral),
        elevator.runElevatorTo(position),
        Commands.waitUntil(elevator::isAtSetpoint),
        coral.scoreCoral(),
        elevator.runElevatorTo(ElevatorPosition.HOME));

    setName("ReefScore(" + position.name() + ")");
  }
}
