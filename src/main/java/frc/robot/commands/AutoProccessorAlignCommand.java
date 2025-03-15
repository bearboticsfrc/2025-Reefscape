package frc.robot.commands;

// import static frc.robot.constants.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceFlipUtil;

public class AutoProccessorAlignCommand extends Command {
  private final double TRANSLATION_P = 6;
  private final double HEADING_P = 10;

  private final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(6);
  private final LinearAcceleration TRANSLATION_MAX_ACCELERATION = MetersPerSecondPerSecond.of(4);

  private final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          TRANSLATION_MAX_VELOCITY.in(MetersPerSecond),
          TRANSLATION_MAX_ACCELERATION.in(MetersPerSecondPerSecond));

  private final FieldCentricFacingAngle DRIVE_TO_POSE = new FieldCentricFacingAngle();

  private final ProfiledPIDController xController =
      new ProfiledPIDController(TRANSLATION_P, 0, 0, TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(TRANSLATION_P, 0, 0, TRANSLATION_CONSTRAINTS);

  private final CommandSwerveDrivetrain drivetrain;

  private Pose2d targetPose = new Pose2d(6, 0.5, Rotation2d.kCCW_90deg);

  public AutoProccessorAlignCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    targetPose = AllianceFlipUtil.apply(targetPose);

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());

    Pose2d currentPose = drivetrain.getState().Pose;

    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    double xVelocity = xController.calculate(currentPose.getX());
    double yVelocity = yController.calculate(currentPose.getY());

    drivetrain.setControl(
        DRIVE_TO_POSE
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withTargetDirection(targetPose.getRotation())
            .withHeadingPID(HEADING_P, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal()
        && yController.atGoal()
        && DRIVE_TO_POSE.HeadingController.atSetpoint();
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
