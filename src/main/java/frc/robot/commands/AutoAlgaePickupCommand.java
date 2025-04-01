package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.VisionConstants.REEF_TAGS_ONLY_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.reef.ReefTagPoses.ScoreSide;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utils.FieldUtils;

/**
 * An autonomous command sequence to align with the reef, pick up algae, and retract. It requires
 * the drivetrain, algae, arm, and elevator subsystems.
 */
public class AutoAlgaePickupCommand extends SequentialCommandGroup {

  private static final Time DRIVE_BACKWARDS_DURATION = Seconds.of(0.5);
  // Note: Velocity is positive, the command negates it for backwards movement.
  private static final LinearVelocity DRIVE_BACKWARDS_VELOCITY = MetersPerSecond.of(1);

  private final CommandSwerveDrivetrain drivetrain;
  private final ElevatorSubsystem elevator;

  /**
   * Creates a new AutoAlgaePickupCommand.
   *
   * @param drivetrain The drivetrain subsystem.
   * @param algae The algae intake subsystem.
   * @param arm The arm subsystem.
   * @param elevator The elevator subsystem.
   */
  public AutoAlgaePickupCommand(
      final CommandSwerveDrivetrain drivetrain,
      final AlgaeSubsystem algae,
      final ArmSubsystem arm,
      final ElevatorSubsystem elevator) {

    this.drivetrain = drivetrain;
    this.elevator = elevator;

    addCommands(
        new AutoReefAlignCommand(drivetrain, ScoreSide.LEFT).alongWith(elevatorRaiseCommand()),
        waitUntilElevatorAtSetpoint(),
        arm.runArmTo(ArmPosition.REEF).alongWith(algae.intakeAlgae()),
        Commands.waitUntil(algae::hasAlgae),
        driveBackwards().alongWith(algae.stopMotor()),
        elevator.runElevatorTo(ElevatorPosition.HOME),
        waitUntilElevatorAtSetpoint());
  }

  /** Creates a command to raise the elevator to a dynamically determined position. */
  private Command elevatorRaiseCommand() {
    return elevator.runElevatorTo(() -> getElevatorPosition(drivetrain.getState().Pose));
  }

  /**
   * Determines the target elevator position based on the nearest reef AprilTag.
   *
   * @param currentPose The robot's current pose.
   * @return The calculated ElevatorPosition (L2 or L3).
   */
  private ElevatorPosition getElevatorPosition(Pose2d currentPose) {
    final int tagId = FieldUtils.findNearestTagId(REEF_TAGS_ONLY_LAYOUT, currentPose);

    // Example: Red reef tags < 12, Blue reef tags >= 12
    // Even/Odd determines L2 vs L3 based on alliance color
    final boolean isEven = tagId % 2 == 0;
    final boolean isRed = tagId < 12;

    return (isRed ^ isEven) ? ElevatorPosition.L3 : ElevatorPosition.L2;
  }

  /** Creates a command that waits until the elevator reports being at its setpoint. */
  private Command waitUntilElevatorAtSetpoint() {
    return Commands.waitUntil(elevator::isAtSetpoint);
  }

  /** Creates a command to drive the robot backwards for a fixed duration and velocity. */
  private Command driveBackwards() {
    // Use RobotCentric request for simple backwards movement
    SwerveRequest.RobotCentric request =
        new SwerveRequest.RobotCentric()
            .withVelocityX(
                -DRIVE_BACKWARDS_VELOCITY.in(MetersPerSecond)); // Negative X is backwards

    return drivetrain
        .runOnce(() -> drivetrain.setControl(request)) // Start driving backwards
        .andThen(Commands.waitTime(DRIVE_BACKWARDS_DURATION)) // Wait
        .andThen(drivetrain.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle()))); // Stop
  }
}
