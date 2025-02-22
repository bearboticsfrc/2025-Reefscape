// Copyright (c) 2025 FRC 4068
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.field;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** */
public class ReefAutoAlignZones {
  public static final Translation2d reefCenter =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static List<ReefAutoAlignZone> zones;

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

      double centerToBranchX = Units.inchesToMeters(30.738);
      double centerToBranchY = Units.inchesToMeters(6.469);

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
                          new Transform2d(centerToBranchX, -centerToBranchY, new Rotation2d()))
                      .getX(),
                  poseDirection
                      .transformBy(
                          new Transform2d(centerToBranchX, -centerToBranchY, new Rotation2d()))
                      .getY()),
              poseDirection.getRotation());

      Transform2d reefFaceOffset =
          new Transform2d(
              Inches.of(1.625).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));
      Transform2d robotBranchScoringOffset =
          new Transform2d(
              Inches.of(17.5).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(0));
      Transform2d rotate180 =
          new Transform2d(
              Inches.of(0).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(180));

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

  public static void main(String[] args) {

    Translation2d point = new Translation2d(5.2, 6);
    for (ReefAutoAlignZone zone : ReefAutoAlignZones.zones) {

      // System.out.println("center " +zone.getCenter()+ "point1 " + zone.getPoint1() + " point2 " +
      // zone.getPoint2());
      System.out.println(
          zone.getCenter().getX()
              + ","
              + zone.getCenter().getY()
              + ","
              + zone.getPoint1().getX()
              + ","
              + zone.getPoint1().getY()
              + ","
              + zone.getPoint2().getX()
              + ","
              + zone.getPoint2().getY());
      /*System.out.println(
                "ScorePose "
                    + zone.getScorePose()
                    + " point "
                    + point
                    + " in zone "
                    + zone.containsPoint(point));
      */
      /*      Pose2d branch = Reef.branchPositions.get(i).get(ReefHeight.L2).toPose2d();
           Pose2d scorePose =
               Reef.branchPositions
                   .get(i)
                   .get(ReefHeight.L2)
                   .toPose2d()
                   .plus(reefFaceOffset)
                   .plus(robotBranchScoringOffset)
                   .plus(rotate180);

           System.out.println("Branch " + i + " pose3d = " + branch);
           System.out.println("score " + i + " pose3d = " + scorePose);
      */ }
    // scoringPose = Reef.branchPositions.get(targetBranch.ordinal()).get(ReefHeight.L2).toPose2d()
    // .plus(robotBranchScoringOffset);
  }
}
