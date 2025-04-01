package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.VisionConstants.CORAL_STATION_TAGS_ONLY_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;
import java.util.Optional;

// Import Supplier

/**
 * A command to align the robot to the nearest coral station scoring pose. It dynamically selects
 * the closest pre-calculated station pose as the target.
 */
public class AutoCoralStationAlignCommand extends DriveToPoseCommand {
  private static final double TRANSLATION_P = 5;
  private static final double TRANSLATION_D = 0;
  private static final double TRANSLATION_I = 0;

  private static final Distance CENTER_TO_CORAL_STATION = Inches.of(16);
  private static final Transform2d POSE_TRANSFORM =
      new Transform2d(new Translation2d(CENTER_TO_CORAL_STATION, Inches.zero()), Rotation2d.kZero);

  private static final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(12);
  private static final LinearAcceleration TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(6);

  private Optional<List<Pose2d>> maybeCoralStationPoses = Optional.empty();
  private CommandSwerveDrivetrain drivetrain;

  /**
   * Creates a new AutoCoralStationAlignCommand.
   *
   * @param drivetrain The drivetrain subsystem.
   */
  public AutoCoralStationAlignCommand(CommandSwerveDrivetrain drivetrain) {
    super(drivetrain, () -> drivetrain.getState().Pose);
    this.drivetrain = drivetrain;

    // Apply specific configurations for this alignment command
    this.withTranslationConstraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION)
        .withTranslationPID(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D)
        .withDebounceDuration(Seconds.of(0))
        .withPoseSupplier(this::getTargetPose);
  }

  private Pose2d getTargetPose() {
    return drivetrain.getState().Pose.nearest(getCoralStationPoses());
  }

  /**
   * Calculates or retrieves the cached list of possible coral station scoring poses. These are
   * derived from the AprilTag layout and a fixed transform.
   *
   * @return A list of Pose2d representing the target scoring locations.
   */
  private List<Pose2d> getCoralStationPoses() {
    if (maybeCoralStationPoses.isPresent()) {
      return maybeCoralStationPoses.get();
    }

    final List<Pose2d> coralStationPoses = calculateCoralStationPoses();
    maybeCoralStationPoses = Optional.of(coralStationPoses);

    return coralStationPoses;
  }

  /**
   * Calculate the transformed coral station poses.
   *
   * @return A list of coral station poses.
   */
  private List<Pose2d> calculateCoralStationPoses() {
    return CORAL_STATION_TAGS_ONLY_LAYOUT.getTags().stream()
        .map(tag -> tag.pose.toPose2d().plus(POSE_TRANSFORM))
        .toList();
  }
}
