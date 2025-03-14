package frc.robot.commands;

import static frc.robot.constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

public class AutoCoralStationAlign extends Command {
  private static final Set<Integer> VALID_TAG_IDS = Set.of(1, 2, 12, 13);

  private static final double P = 10;
  private static final double I = 0;
  private static final double D = 0;

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

    this.poseToRotation = generatePoseToRotation(VALID_TAG_IDS);
    this.tagPoses.addAll(poseToRotation.keySet());

    addRequirements(drivetrain);
  }

  /**
   * Generate a mapping of all April Tag poses' which IDs are in {@code validTagIDs} to the tag's
   * refleted rotation.
   *
   * @param validTagIDs The tag IDs to use.
   * @return The mapping.
   */
  private Map<Pose2d, Rotation2d> generatePoseToRotation(Collection<Integer> validTagIDs) {
    return APRIL_TAG_FIELD_LAYOUT.getTags().stream()
        .filter(tag -> validTagIDs.contains(tag.ID))
        .collect(Collectors.toMap(this::mapKey, this::mapValue));
  }

  /**
   * Map the April Tag to its pose.
   *
   * @param tag The April Tag.
   * @return The April Tag pose
   */
  private Pose2d mapKey(AprilTag tag) {
    return tag.pose.toPose2d();
  }

  /**
   * Map the April Tag to its inverse rotation {@code (180 - AprilTagPoseRotation)}.
   *
   * @param tag The April Tag.
   * @return The April Tag's reflected rotation.
   */
  private Rotation2d mapValue(AprilTag tag) {
    return tag.pose.toPose2d().getRotation().minus(Rotation2d.k180deg);
  }

  /**
   * Initialize this command by setting the target rotation to the nearest April Tag's reflected
   * rotation.
   */
  @Override
  public void initialize() {
    Pose2d nearestTag = drivetrain.getState().Pose.nearest(tagPoses);
    targetRotation = poseToRotation.get(nearestTag);

    swerveRequest.TargetDirection = targetRotation;
  }

  /**
   * Apply the swerve request using {@code xSupplier} and {@code ySupplier} as input to {@code
   * SwerveRequest.withVelocityX} and {@code SwerveRequest.withVelocityY}, respectivly.
   */
  @Override
  public void execute() {
    drivetrain.setControl(
        swerveRequest
            .withVelocityX(xSupplier.getAsDouble())
            .withVelocityY(ySupplier.getAsDouble())
            .withHeadingPID(P, I, D));
  }

  /** Apply a {@link SwerveRequest.Idle} to the drivetrain on command end. */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
