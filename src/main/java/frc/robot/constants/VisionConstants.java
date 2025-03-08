package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import frc.robot.vision.VisionCamera;

public class VisionConstants {
  public static final Distance CULLING_DISTANCE = Meters.of(2.5);
  public static final double CULLING_AMBIGUITY = 0.2;

  private static final String FRONT_LEFT_CAMERA_NAME = "OV9281FrontLeft";
  private static final String FRONT_RIGHT_CAMERA_NAME = "OV9281FrontRight";
  private static final String REAR_CAMERA_NAME = "OV9281Rear";

  private static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA =
      new Transform3d(
          new Translation3d(.1524, 0.254, 0.4617),
          new Rotation3d(Radians.zero(), Degrees.of(22), Degrees.zero()));

  private static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA =
      new Transform3d(
          new Translation3d(.1524, -0.254, 0.4617),
          new Rotation3d(Radians.zero(), Degrees.of(22), Degrees.zero()));

  private static final Transform3d ROBOT_TO_REAR_CAMERA =
      new Transform3d(
          new Translation3d(-.019, 0.0, 0.957),
          new Rotation3d(Radians.zero(), Degrees.of(160), Degrees.of(0)));

  public static final VisionCamera FRONT_LEFT_CAMERA =
      new VisionCamera(FRONT_LEFT_CAMERA_NAME, ROBOT_TO_FRONT_LEFT_CAMERA);
  public static final VisionCamera FRONT_RIGHT_CAMERA =
      new VisionCamera(FRONT_RIGHT_CAMERA_NAME, ROBOT_TO_FRONT_RIGHT_CAMERA);
  public static final VisionCamera REAR_CAMERA =
      new VisionCamera(REAR_CAMERA_NAME, ROBOT_TO_REAR_CAMERA);

  // The standard deviations of our vision estimated poses, which affect correction rate
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
      VecBuilder.fill(.8, .8, Math.toRadians(30));
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
      VecBuilder.fill(0.1, 0.1, Math.toRadians(20));

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
}
