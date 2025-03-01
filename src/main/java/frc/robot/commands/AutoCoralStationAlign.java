package frc.robot.commands;

import static frc.robot.constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class AutoCoralStationAlign extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private final FieldCentricFacingAngle swerveRequest = new FieldCentricFacingAngle();

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final Map<Pose2d, Rotation2d> poseToRotation;
  private List<Pose2d> tagPoses = new ArrayList<>();

  private Rotation2d targetRotation;

  public AutoCoralStationAlign(
      CommandSwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drivetrain = drivetrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    this.poseToRotation = getPoseToRotation();

    addRequirements(drivetrain);
  }

  private Map<Pose2d, Rotation2d> getPoseToRotation() {
    Map<Pose2d, Rotation2d> poseToRotation = new HashMap<>();

    for (AprilTag tag : APRIL_TAG_FIELD_LAYOUT.getTags()) {
      if (tag.ID != 1 || tag.ID != 2 || tag.ID != 12 || tag.ID != 13) {
        continue;
      }

      Pose2d tagPose = tag.pose.toPose2d();

      tagPoses.add(tagPose);
      poseToRotation.put(tagPose, Rotation2d.fromDegrees(180 - tagPose.getRotation().getDegrees()));
    }

    return poseToRotation;
  }

  @Override
  public void initialize() {
    targetRotation = poseToRotation.get(drivetrain.getState().Pose.nearest(tagPoses));
    swerveRequest.TargetDirection = targetRotation;
  }

  @Override
  public void execute() {
    drivetrain.setControl(
        swerveRequest
            .withVelocityX(xSupplier.getAsDouble())
            .withVelocityY(ySupplier.getAsDouble()));
  }
}
