package frc.robot.commands;

import static frc.robot.constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

public class AutoCoralStationAlign extends Command {
  private static final Set<Integer> VALID_TAG_IDS = Set.of(1, 2, 12, 13);

  private final CommandSwerveDrivetrain drivetrain;

  private final FieldCentricFacingAngle swerveRequest = new FieldCentricFacingAngle();

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final Map<Pose2d, Rotation2d> poseToRotation;
  private final List<Pose2d> tagPoses = new ArrayList<>();

  private Rotation2d targetRotation;

  public AutoCoralStationAlign(
      CommandSwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drivetrain = drivetrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    this.poseToRotation = generatePoseToRotation();
    this.tagPoses.addAll(poseToRotation.keySet());

    addRequirements(drivetrain);
  }

  private Map<Pose2d, Rotation2d> generatePoseToRotation() {
    return APRIL_TAG_FIELD_LAYOUT.getTags().stream()
        .filter(tag -> VALID_TAG_IDS.contains(tag.ID))
        .collect(Collectors.toMap(this::mapKey, this::mapValue));
  }

  private Pose2d mapKey(AprilTag tag) {
    return tag.pose.toPose2d();
  }

  private Rotation2d mapValue(AprilTag tag) {
    return Rotation2d.k180deg.minus(tag.pose.toPose2d().getRotation());
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
