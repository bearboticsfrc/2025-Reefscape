package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.VisionConstants.REEF_TAGS_ONLY_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.reef.ReefTagPoses.ScoreSide;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utils.FieldUtils;

public class AutoAlgaePickupCommand {
  private static Time DRIVE_BACKWARDS_DURATION = Seconds.of(0.5);

  private static SwerveRequest.RobotCentric swerveRequest =
      new SwerveRequest.RobotCentric().withVelocityX(-1);

  public static Command get(
      CommandSwerveDrivetrain drivetrain,
      AlgaeSubsystem algae,
      ArmSubsystem arm,
      ElevatorSubsystem elevator) {
    return AutoReefAlignCommand.get(drivetrain, ScoreSide.LEFT)
        .andThen(elevatorRaiseCommand(drivetrain, elevator))
        .andThen(waitUntilElevatorAtSetpoint(elevator))
        .andThen(arm.runArmTo(ArmPosition.REEF).alongWith(algae.intakeAlgae()))
        .andThen(Commands.waitUntil(algae::hasAlgae))
        .andThen(driveBackwards(drivetrain))
        .andThen(elevator.runElevatorTo(ElevatorPosition.L2))
        .andThen(waitUntilElevatorAtSetpoint(elevator));
  }

  private static Command elevatorRaiseCommand(
      CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator) {
    return elevator.runElevatorTo(() -> getElevatorPosition(drivetrain.getState().Pose));
  }

  private static ElevatorPosition getElevatorPosition(Pose2d currentPose) {
    final int tagId = FieldUtils.findNearestTagId(REEF_TAGS_ONLY_LAYOUT, currentPose);

    final boolean isEven = tagId % 2 == 0;
    final boolean isRed = tagId < 12;

    if (isRed) {
      return isEven ? ElevatorPosition.L2 : ElevatorPosition.L3;
    } else {
      return isEven ? ElevatorPosition.L3 : ElevatorPosition.L2;
    }
  }

  private static Command waitUntilElevatorAtSetpoint(ElevatorSubsystem elevator) {
    return Commands.waitUntil(elevator::isAtSetpoint);
  }

  private static Command driveBackwards(CommandSwerveDrivetrain drivetrain) {
    return drivetrain
        .runOnce(() -> drivetrain.setControl(swerveRequest))
        .andThen(Commands.waitTime(DRIVE_BACKWARDS_DURATION))
        .andThen(drivetrain.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle())));
  }
}
