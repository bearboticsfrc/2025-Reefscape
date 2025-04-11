package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;

public class ProccessorScoreCommand extends SequentialCommandGroup {
  private static final Time SCORE_DELAY = Seconds.of(0.5);

  public ProccessorScoreCommand(AlgaeSubsystem algae, ArmSubsystem arm) {
    addCommands(
        arm.runArmTo(ArmPosition.HOME),
        Commands.waitUntil(arm::isAtSetpoint),
        Commands.waitTime(SCORE_DELAY),
        algae.scoreProcessor(),
        algae.stopMotor());

    addRequirements(algae, arm);
  }
}
