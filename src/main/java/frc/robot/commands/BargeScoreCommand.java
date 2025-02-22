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

public class BargeScoreCommand {
  private static final Time SCORE_WAIT = Milliseconds.of(250);

  public static Command raise(ElevatorSubsystem elevator, ArmSubsystem arm, AlgaeSubsystem algae) {
    return elevator
        .runElevatorTo(ElevatorPosition.L4)
        .andThen(Commands.waitUntil(elevator::isAtSetpoint))
        .andThen(arm.runArmTo(ArmPosition.BARGE))
        .andThen(Commands.waitUntil(arm::isAtSetpoint))
        .andThen(Commands.waitTime(SCORE_WAIT))
        .andThen(algae.scoreAlgae())
        .andThen(arm.runArmTo(ArmPosition.HOME))
        .andThen(Commands.waitUntil(arm::isAtSetpoint))
        .andThen(elevator.runElevatorTo(ElevatorPosition.HOME))
        .andThen(Commands.waitUntil(elevator::isAtSetpoint));
  }

  public static Command lower(ElevatorSubsystem elevator, ArmSubsystem arm, AlgaeSubsystem algae) {
    return arm.runArmTo(ArmPosition.HOME)
        .andThen(Commands.waitUntil(arm::isAtSetpoint))
        .andThen(elevator.runElevatorTo(ElevatorPosition.HOME))
        .andThen(Commands.waitUntil(elevator::isAtSetpoint))
        .alongWith(algae.stopMotor());
  }
}
