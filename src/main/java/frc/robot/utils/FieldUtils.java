package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.stream.Collectors;

/**
 * A utility class that provides methods for interpreting and working with field data and AprilTag
 * layouts.
 */
public class FieldUtils {

  /**
   * Finds the AprilTag pose closest to the provided pose.
   *
   * <p>This method converts each tag's pose in the {@code fieldLayout} into a {@link Pose2d}, then
   * uses the {@code currentPose} to determine which pose is closest.
   *
   * @param fieldLayout the layout containing all AprilTags to iterate through.
   * @param pose the provided pose.
   * @return the {@link Pose2d} of the nearest AprilTag.
   */
  public static Pose2d findNearestTagPose(AprilTagFieldLayout fieldLayout, Pose2d pose) {
    return pose.nearest(
        fieldLayout.getTags().stream()
            .map(tag -> tag.pose.toPose2d())
            .collect(Collectors.toList()));
  }
}
