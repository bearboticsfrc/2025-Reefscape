/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {
  private final List<PhotonCamera> cameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> photonEstimators = new ArrayList<>();

  @Logged(name = "Target Poses")
  private final List<Pose2d> targetPoses = new ArrayList<>();

  public Vision(List<VisionCamera> visionCameras) {
    for (VisionCamera visionCamera : visionCameras) {
      PhotonCamera camera = new PhotonCamera(visionCamera.getName());

      PhotonPoseEstimator photonEstimator =
          new PhotonPoseEstimator(
              APRIL_TAG_FIELD_LAYOUT,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              visionCamera.getTransform());

      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      cameras.add(camera);
      photonEstimators.add(photonEstimator);
    }
  }

  /**
   * Get the latest photon result from the camera
   *
   * @return The result
   */
  public Optional<PhotonPipelineResult> getLatestResult(PhotonCamera camera) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (results.size() < 1) {
      return Optional.empty();
    }

    return Optional.of(results.get(0));
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
    List<Optional<PhotonPipelineResult>> results = new ArrayList<>();

    for (PhotonCamera camera : cameras) {
      results.add(getLatestResult(camera));
    }

    List<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
    targetPoses.clear();

    for (int i = 0; i < photonEstimators.size(); i++) {
      Optional<PhotonPipelineResult> maybeResult = results.get(i);

      if (maybeResult.isEmpty()) {
        continue;
      }

      PhotonPipelineResult result = maybeResult.get();

      if (!result.hasTargets() || isTooFar(result) || isTooAmbiguous(result)) {
        continue;
      }

      PhotonPoseEstimator photonEstimator = photonEstimators.get(i);
      Optional<EstimatedRobotPose> maybeEstimatedPose = photonEstimator.update(maybeResult.get());

      if (maybeEstimatedPose.isEmpty()) {
        continue;
      }

      EstimatedRobotPose estimatedPose = maybeEstimatedPose.get();

      targetPoses.add(estimatedPose.estimatedPose.toPose2d());
      estimatedPoses.add(estimatedPose);
    }

    return estimatedPoses;
  }

  private boolean isTooFar(PhotonPipelineResult result) {
    return result.getBestTarget().bestCameraToTarget.getMeasureX().gt(CULLING_DISTANCE);
  }

  private boolean isTooAmbiguous(PhotonPipelineResult result) {
    return result.getBestTarget().poseAmbiguity > CULLING_AMBIGUITY;
  }
}
