package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.AllianceFlipUtil;
import java.util.Arrays;

public class NearestReefZone {
  public double tangentialBranchOffset;

  public Transform2d getTanUnitVectorToFace(Pose2d face) {
    this.tangentialBranchOffset= AllianceFlipUtil.shouldFlip()? Units.inchesToMeters(6.469): Units.inchesToMeters(6.469) *-1; // meters; // meters
    Translation2d translation =
        new Translation2d(
            Math.cos(face.getRotation().getRadians()), Math.sin(face.getRotation().getRadians()));
    return new Transform2d(translation, new Rotation2d());
  }

  public Pose2d[] getBranches(Pose2d face) {
    Pose2d leftBranch = face.plus(getTanUnitVectorToFace(face).times(tangentialBranchOffset * -1));
    Pose2d rightBranch = face.plus(getTanUnitVectorToFace(face).times(tangentialBranchOffset));
    return new Pose2d[] {leftBranch, rightBranch};
  }

  public Pose2d getNearestFace(Pose2d pose) {
    Pose2d[] faces = FieldConstants.Reef.faces;
    if (AllianceFlipUtil.shouldFlip()) {
      for (Pose2d face : faces) {
        AllianceFlipUtil.apply(face);
      }
    }

    return pose.nearest(Arrays.asList(faces));
  }

  public Pose2d getNearestBranch(Pose2d pose) {
    Pose2d nearestFace = getNearestFace(pose);
    Pose2d[] branches = getBranches(nearestFace);
    return pose.nearest(Arrays.asList(branches));
  }

  public Pose2d getLeftBranch(Pose2d pose) {
    Pose2d[] branches = getBranches(getNearestFace(pose));
    return branches[0];
  }

  public Pose2d getRightBranch(Pose2d pose) {
    Pose2d[] branches = getBranches(getNearestFace(pose));
    return branches[1];
  }
}
