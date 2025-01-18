package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

/** Represents a vision camera on the robot. */
public class VisionCamera {
  private String niceName;
  private PhotonCamera photonCamera;
  private Transform3d robotToCameraTransform;

  /**
   * Constructs a VisionCamera with a user-friendly name, the PhotonCamera instance, and the
   * transformation from the robot's coordinate system to the camera's coordinate system.
   *
   * @param niceName A user-friendly name for the camera, used for identification purposes.
   * @param photonCamera The PhotonCamera instance associated with this VisionCamera.
   * @param robotToCameraTransform The transformation from the robot's coordinate system to the
   *     camera's coordinate system.
   */
  public VisionCamera(
      String niceName, PhotonCamera photonCamera, Transform3d robotToCameraTransform) {
    this.niceName = niceName;
    this.photonCamera = photonCamera;
    this.robotToCameraTransform = robotToCameraTransform;
  }

  /**
   * Gets the user-friendly name of the camera.
   *
   * @return The user-friendly name of the camera.
   */
  public String getNiceName() {
    return niceName;
  }

  /**
   * Gets the transformation from the robot's coordinate system to the camera's coordinate system.
   *
   * @return The transformation from the robot to the camera.
   */
  public Transform3d getRobotToCameraTransform() {
    return robotToCameraTransform;
  }

  /**
   * Gets the PhotonCamera instance associated with this VisionCamera.
   *
   * @return The PhotonCamera instance.
   */
  public PhotonCamera getPhotonCamera() {
    return photonCamera;
  }
}
