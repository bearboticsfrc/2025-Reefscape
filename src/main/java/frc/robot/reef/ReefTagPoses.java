// Copyright (c) 2025 FRC 4068
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.reef;

import static frc.robot.constants.VisionConstants.REEF_TAGS_ONLY_LAYOUT;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/** */
public class ReefTagPoses {
  private static Optional<Map<Pose2d, ReefScorePose>> maybeTagPoses = Optional.empty();

  public static Pose2d getNearestScoringPose(Pose2d currentPose, ScoreSide side) {
    ArrayList<Pose2d> sidePoses = new ArrayList<>();

    for (ReefScorePose reefScorePose : getTagPoses().values()) {
      switch (side) {
        case LEFT:
          sidePoses.add(reefScorePose.getLeftPose());
          break;
        case RIGHT:
          sidePoses.add(reefScorePose.getRightPose());
          break;
      }
    }

    return currentPose.nearest(sidePoses);
  }

  private static Map<Pose2d, ReefScorePose> getTagPoses() {
    if (maybeTagPoses.isPresent()) {
      return maybeTagPoses.get();
    }

    Map<Pose2d, ReefScorePose> tagPoses = new HashMap<>();

    for (AprilTag tag : REEF_TAGS_ONLY_LAYOUT.getTags()) {
      Pose2d tagPose = tag.pose.toPose2d();
      tagPoses.put(tagPose, new ReefScorePose(tagPose));
    }

    maybeTagPoses = Optional.of(tagPoses);
    return tagPoses;
  }

  public enum ScoreSide {
    LEFT,
    RIGHT,
  }
}
