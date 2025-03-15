package frc.robot.commands;

// import static frc.robot.constants.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class AutoProccessorAlignCommand extends Command {
  private static final Set<Integer> VALID_TAG_IDS = Set.of(3, 16);
  private static final Transform2d TAG_TO_ROBOT_TRANSFORM =
      new Transform2d(0.5, -0.15, Rotation2d.k180deg);

  private final double TRANSLATION_P = 6;
  private final double HEADING_P = 2;

  private final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(4);
  private final LinearAcceleration TRANSLATION_MAX_ACCELERATION = MetersPerSecondPerSecond.of(2);

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

  private final Map<Pose2d, Pose2d> poseToPose;
  private final List<Pose2d> tagPoses = new ArrayList<>();

  private Pose2d targetPose;

  public AutoProccessorAlignCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    this.poseToPose = generatePoseToRotation();
    this.tagPoses.addAll(poseToPose.keySet());

    addRequirements(drivetrain);
  }

  private Map<Pose2d, Pose2d> generatePoseToRotation() {
    return APRIL_TAG_FIELD_LAYOUT.getTags().stream()
        .filter(tag -> VALID_TAG_IDS.contains(tag.ID))
        .collect(Collectors.toMap(this::mapKey, this::mapValue));
  }

  private Pose2d mapKey(AprilTag tag) {
    return tag.pose.toPose2d();
  }

  private Pose2d mapValue(AprilTag tag) {
    return tag.pose.toPose2d().plus(TAG_TO_ROBOT_TRANSFORM);
  }

  @Override
  public void initialize() {
    targetPose = poseToPose.get(drivetrain.getState().Pose.nearest(tagPoses));

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
