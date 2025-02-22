// Copyright (c) 2025 FRC 4068
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.field;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

/** */
public class ReefAutoAlignZones {
  public static final Translation2d reefCenter =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static List<ReefAutoAlignZone> zones;

  public static HashMap<Pose2d, TagPose> tagPoses = new HashMap<>();

  static {
    zones = new ArrayList<>();
    for (int face = 0; face < 6; face++) {
      Pose2d poseDirection = new Pose2d(reefCenter, Rotation2d.fromDegrees(180 + (60 * face)));
      double adjustX = 3.15; // meters
      double adjustY = 1.80; // meters

      Translation2d leftPoint =
          new Translation2d(
              poseDirection.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())).getX(),
              poseDirection
                  .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                  .getY());
      Translation2d middlePoint =
          new Translation2d(
              poseDirection.transformBy(new Transform2d(adjustX, 0, new Rotation2d())).getX(),
              poseDirection.transformBy(new Transform2d(adjustX, 0, new Rotation2d())).getY());
      Translation2d rightPoint =
          new Translation2d(
              poseDirection
                  .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                  .getX(),
              poseDirection
                  .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                  .getY());

      Distance centerToBranchX = Inches.of(30.738);
      Distance centerToBranchY = Inches.of(6.469);

      Pose2d rightBranch =
          new Pose2d(
              new Translation2d(
                  poseDirection
                      .transformBy(
                          new Transform2d(centerToBranchX, centerToBranchY, new Rotation2d()))
                      .getX(),
                  poseDirection
                      .transformBy(
                          new Transform2d(centerToBranchX, centerToBranchY, new Rotation2d()))
                      .getY()),
              poseDirection.getRotation());

      Pose2d leftBranch =
          new Pose2d(
              new Translation2d(
                  poseDirection
                      .transformBy(
                          new Transform2d(
                              centerToBranchX, centerToBranchY.unaryMinus(), new Rotation2d()))
                      .getX(),
                  poseDirection
                      .transformBy(
                          new Transform2d(
                              centerToBranchX, centerToBranchY.unaryMinus(), new Rotation2d()))
                      .getY()),
              poseDirection.getRotation());

      Transform2d reefFaceOffset =
          new Transform2d(Inches.of(1.625), Inches.of(0), Rotation2d.fromDegrees(0));
      Transform2d robotBranchScoringOffset =
          new Transform2d(Inches.of(17.5), Inches.of(0), Rotation2d.fromDegrees(0));
      Transform2d rotate180 =
          new Transform2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(180));

      Pose2d leftScorePose =
          rightBranch.plus(reefFaceOffset).plus(robotBranchScoringOffset).plus(rotate180);
      Pose2d rightScorePose =
          leftBranch.plus(reefFaceOffset).plus(robotBranchScoringOffset).plus(rotate180);

      zones.add(
          new ReefAutoAlignZone(
              reefCenter, leftPoint, middlePoint, poseDirection.getRotation(), leftScorePose));
      zones.add(
          new ReefAutoAlignZone(
              reefCenter, rightPoint, middlePoint, poseDirection.getRotation(), rightScorePose));

      for (AprilTag tag : APRIL_TAG_FIELD_LAYOUT.getTags()) {
        if ((tag.ID < 6 || tag.ID > 11) && (tag.ID < 15 || tag.ID > 22)) {
          continue;
        }

        TagPose tagPose = new TagPose();

        tagPose.left =
            tag.pose
                .toPose2d()
                .plus(
                    new Transform2d(
                        new Translation2d(Inches.of(18), centerToBranchY), Rotation2d.k180deg));

        tagPose.right =
            tag.pose
                .toPose2d()
                .plus(
                    new Transform2d(
                        new Translation2d(Inches.of(18), centerToBranchY.unaryMinus()),
                        Rotation2d.k180deg));

        tagPoses.put(tag.pose.toPose2d(), tagPose);
      }
    }
  }

  public static Optional<ReefAutoAlignZone> inZone(Translation2d point) {
    for (ReefAutoAlignZone zone : ReefAutoAlignZones.zones) {
      if (zone.containsPoint(point)) {
        return Optional.of(zone);
      }
    }
    return Optional.empty();
  }
}
