package frc.robot.commands;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;

/**
 * Provides command sequences for performing a barge score, including both raising and lowering the
 * manipulator subsystems.
 */
public class BargeScoreCommand {
  private static final Time SCORE_WAIT = Milliseconds.of(200);

  /**
   * Creates a command sequence to perform the "raise" portion of a barge score.
   *
   * <p>This command sequence does the following:
   *
   * <ol>
   *   <li>Moves the elevator to the L4 scoring position.
   *   <li>Waits until the elevator reaches its setpoint.
   *   <li>Moves the arm to the BARGE scoring position.
   *   <li>Waits until the arm reaches its setpoint.
   *   <li>Waits for an additional 500 milliseconds to allow for stabilization.
   *   <li>Activates the algae scoring mechanism.
   *   <li>Moves the arm back to its HOME position.
   *   <li>Waits until the arm is at the HOME setpoint.
   *   <li>Moves the elevator back to its HOME position.
   *   <li>Waits until the elevator reaches its HOME setpoint.
   * </ol>
   *
   * @param elevator the elevator subsystem to control
   * @param arm the arm subsystem to control
   * @param algae the algae subsystem responsible for scoring
   * @return a command that executes the complete raise sequence for barge scoring
   */
  public static Command raise(ElevatorSubsystem elevator, ArmSubsystem arm, AlgaeSubsystem algae) {
    return elevator
        .runElevatorTo(ElevatorPosition.L4)
        .alongWith(arm.runArmTo(ArmPosition.BARGE))
        .andThen(Commands.waitUntil(() -> elevatorAndArmAtSetpoint(elevator, arm)))
        .andThen(Commands.waitTime(SCORE_WAIT))
        .andThen(algae.scoreAlgae())
        .andThen(
            elevator.runElevatorTo(ElevatorPosition.HOME).alongWith(arm.runArmTo(ArmPosition.HOME)))
        .andThen(Commands.waitUntil(() -> elevatorAndArmAtSetpoint(elevator, arm)));
  }

  /**
   * Creates a command sequence to perform the "lower" portion of a barge score.
   *
   * <p>This command sequence does the following:
   *
   * <ol>
   *   <li>Moves the arm to its HOME position.
   *   <li>Waits until the arm reaches its setpoint.
   *   <li>Moves the elevator to its HOME position.
   *   <li>Waits until the elevator reaches its setpoint.
   *   <li>Stops the algae mechanism concurrently with the above actions.
   * </ol>
   *
   * @param elevator the elevator subsystem to control
   * @param arm the arm subsystem to control
   * @param algae the algae subsystem responsible for scoring
   * @return a command that executes the complete lower sequence for barge scoring
   */
  public static Command lower(ElevatorSubsystem elevator, ArmSubsystem arm, AlgaeSubsystem algae) {
    return arm.runArmTo(ArmPosition.HOME)
        .andThen(Commands.waitUntil(arm::isAtSetpoint))
        .andThen(elevator.runElevatorTo(ElevatorPosition.HOME))
        .andThen(Commands.waitUntil(elevator::isAtSetpoint))
        .alongWith(algae.stopMotor());
  }

  public static boolean elevatorAndArmAtSetpoint(ElevatorSubsystem elevator, ArmSubsystem arm) {
    return elevator.isAtSetpoint() && arm.isAtSetpoint();
  }
}
