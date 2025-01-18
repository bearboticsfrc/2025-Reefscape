// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Subsystem for object detection using a PhotonCamera. */
public class ObjectDetectionSubsystem extends SubsystemBase {
  private PhotonCamera photonCamera;
  private boolean driverCameraMode;

  /**
   * Constructs an ObjectDetectionSubsystem with the specified camera name.
   *
   * @param cameraName The name of the PhotonCamera used for object detection.
   */
  public ObjectDetectionSubsystem(String cameraName) {
    photonCamera = new PhotonCamera(cameraName);
  }

  /** Toggles the camera mode between driver mode and vision processing mode. */
  public void toggleDriverCamera() {
    photonCamera.setDriverMode(driverCameraMode ^= true);
  }

  /**
   * Checks if the camera currently has a vision target in view.
   *
   * @return True if there is at least one target in view, false otherwise.
   */
  public boolean hasTargetInView() {
    return photonCamera.getLatestResult().hasTargets();
  }

  /**
   * Retrieves the best target currently in view of the camera, if any.
   *
   * @return An Optional containing the best PhotonTrackedTarget, or an empty Optional if no targets
   *     are detected.
   */
  public Optional<PhotonTrackedTarget> getBestTarget() {
    return hasTargetInView()
        ? Optional.of(photonCamera.getLatestResult().getBestTarget())
        : Optional.empty();
  }

  /**
   * Retrieves the Transform3d from the camera to the best target, if any targets are in view.
   *
   * @return An Optional containing the Transform3d to the best target, or an empty Optional if no
   *     targets are detected.
   */
  public Optional<Transform3d> getTransformToBestTarget() {
    return hasTargetInView()
        ? Optional.of(getBestTarget().get().getBestCameraToTarget())
        : Optional.empty();
  }
}
