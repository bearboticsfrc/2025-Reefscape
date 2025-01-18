package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.location.FieldPositions;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Runnable implementation for performing pose estimation using AprilTag detections from
 * PhotonVision
 */
public class EstimationRunnable implements Runnable {
  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;

  private final AprilTagFieldLayout layout;

  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<>();

  /**
   * Constructs a new EstimationRunnable with the specified camera configuration.
   *
   * @param name The name of the PhotonCamera used for vision processing.
   * @param camera The VisionCamera configuration.
   */
  public EstimationRunnable(String name, VisionCamera camera) {
    this.layout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    this.layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    this.photonCamera = camera.getPhotonCamera();

    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            //  photonCamera,
            camera.getRobotToCameraTransform());
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * The main run method executed by the runnable. It retrieves the latest results from the
   * PhotonCamera, filters out targets based on ambiguity and distance, and updates the estimated
   * robot pose accordingly. Only valid poses within the field boundaries are considered.
   */
  @Override
  public void run() {
    PhotonPipelineResult photonResults = photonCamera.getLatestResult();

    photonResults.targets.removeIf(
        t -> t.getPoseAmbiguity() > APRILTAG_AMBIGUITY_THRESHOLD || isTargetTooFarAway(t));

    if (!photonResults.hasTargets()) return;

    photonPoseEstimator
        .update(photonResults)
        .ifPresent(
            estimatedRobotPose -> {
              Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
              if (estimatedPose.getX() > 0.0
                  && estimatedPose.getX() <= FieldPositions.FIELD_LENGTH
                  && estimatedPose.getY() > 0.0
                  && estimatedPose.getY() <= FieldPositions.FIELD_WIDTH) {
                atomicEstimatedRobotPose.set(estimatedRobotPose);
              }
            });
  }

  /**
   * Determines whether a detected target is too far away based on a predefined culling distance.
   *
   * @param target The PhotonTrackedTarget to evaluate.
   * @return True if the target is beyond the culling distance, false otherwise.
   */
  private boolean isTargetTooFarAway(PhotonTrackedTarget target) {
    Transform3d t3d = target.getBestCameraToTarget();
    return (Math.hypot(t3d.getX(), t3d.getY())) > APRILTAG_CULL_DISTANCE;
  }

  /**
   * Retrieves the latest estimated robot pose and resets the internal reference to null. This
   * method is designed to return each pose estimate only once to ensure each estimate is processed
   * in a timely manner.
   *
   * @return The latest estimated robot pose, or null if there is no new estimate since the last
   *     call.
   */
  public EstimatedRobotPose getLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }
}
