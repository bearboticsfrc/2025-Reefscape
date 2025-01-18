package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.text.DecimalFormat;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class StringFormatting {

  private static final DecimalFormat df = new DecimalFormat("0.00");
  private static final DecimalFormat df_millis = new DecimalFormat("0.00");

  public static String estimatedRobotPoseToString(EstimatedRobotPose robotPose) {
    StringBuilder sb = new StringBuilder();
    sb.append(poseToString(robotPose.estimatedPose));
    sb.append(" ");
    sb.append("timestamp_age=");
    sb.append(
        df_millis.format((RobotController.getFPGATime() * 1000000) - robotPose.timestampSeconds));
    sb.append(" ");
    sb.append(tagsToString(robotPose.targetsUsed));
    return sb.toString();
  }

  public static String poseToString(Pose3d pose) {

    StringBuilder sb = new StringBuilder();
    sb.append("Pose[");
    sb.append(df.format(pose.getX()));
    sb.append(",");
    sb.append(df.format(pose.getY()));
    sb.append(",");
    sb.append(df.format(pose.getZ()));
    sb.append("@");
    sb.append(df.format(Math.toDegrees(pose.getRotation().getAngle())));
    sb.append("]");

    return sb.toString();
  }

  public static String getPose3dDoubleArrayString(Pose3d pose) {
    StringBuilder sb = new StringBuilder();
    sb.append("[");
    sb.append(df.format(pose.getX()));
    sb.append(",");
    sb.append(df.format(pose.getY()));
    sb.append(",");
    sb.append(df.format(pose.getZ()));
    sb.append(",");
    sb.append(pose.getRotation().getQuaternion().getW());
    sb.append(",");
    sb.append(pose.getRotation().getQuaternion().getX());
    sb.append(",");
    sb.append(pose.getRotation().getQuaternion().getY());
    sb.append(",");
    sb.append(pose.getRotation().getQuaternion().getZ());
    sb.append("]");

    return sb.toString();
  }

  public static String poseToString(Pose2d pose) {

    StringBuilder sb = new StringBuilder();
    sb.append("Pose[");
    sb.append(df.format(pose.getX()));
    sb.append(",");
    sb.append(df.format(pose.getY()));
    sb.append("@");
    sb.append(df.format(pose.getRotation().getDegrees()));
    sb.append("]");

    return sb.toString();
  }

  public static String tagsToString(List<PhotonTrackedTarget> tags) {
    StringBuilder sb = new StringBuilder();
    sb.append("TagsUsed[");
    for (PhotonTrackedTarget target : tags) {
      sb.append(" id=");
      sb.append(target.getFiducialId());
      sb.append(" ambiguity=");
      sb.append(target.getPoseAmbiguity());
      sb.append(" distance=");
      sb.append(target.getBestCameraToTarget().getTranslation().getNorm());
      sb.append(" x=");
      sb.append(target.getBestCameraToTarget().getTranslation().getX());
      sb.append(" y=");
      sb.append(target.getBestCameraToTarget().getTranslation().getY());
    }
    sb.append("]");
    return sb.toString();
  }

  public static String transformToString(Transform3d transform) {

    StringBuilder sb = new StringBuilder();
    sb.append("Transform3d[");
    sb.append(df.format(transform.getX()));
    sb.append(",");
    sb.append(df.format(transform.getY()));
    sb.append(",");
    sb.append(df.format(transform.getZ()));
    sb.append("@");
    sb.append(df.format(Units.radiansToDegrees(transform.getRotation().getAngle())));
    sb.append("]");

    return sb.toString();
  }

  public static String translation2dToString(Translation2d translation) {

    StringBuilder sb = new StringBuilder();
    sb.append("Translation2d[");
    sb.append(df.format(translation.getX()));
    sb.append(",");
    sb.append(df.format(translation.getY()));
    // sb.append("@");
    // sb.append(df.format(translation.getAngle().getDegrees()));
    sb.append("]");

    return sb.toString();
  }
}
