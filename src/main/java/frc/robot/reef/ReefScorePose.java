package frc.robot.reef;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class ReefScorePose {
  private final Distance CENTER_TO_BRANCH_Y = Inches.of(6.469);

  private final Transform2d LEFT_POSE_TRANSFORM =
      new Transform2d(
          new Translation2d(Inches.of(18), CENTER_TO_BRANCH_Y.unaryMinus()), Rotation2d.k180deg);
  private final Transform2d RIGHT_POSE_TRANSFORM =
      new Transform2d(new Translation2d(Inches.of(18), CENTER_TO_BRANCH_Y), Rotation2d.k180deg);

  private Pose2d leftPose;
  private Pose2d rightPose;

  public ReefScorePose(Pose2d tagPose) {
    leftPose = tagPose.plus(LEFT_POSE_TRANSFORM);
    rightPose = tagPose.plus(RIGHT_POSE_TRANSFORM);
  }

  public Pose2d getLeftPose() {
    return leftPose;
  }

  public Pose2d getRightPose() {
    return rightPose;
  }
}
