package frc.robot.constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

  public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA =
      new Transform3d(
          new Translation3d(.095, 0.295, 0.31),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0)));

  public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA =
      new Transform3d(
          new Translation3d(.412625, -0.2242, 0.53026),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(-27.0), Math.toRadians(-31.0)));

  public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA =
      new Transform3d(
          new Translation3d(.0178, 0.3072, 0.40311),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(-30.0), Math.toRadians(166.1)));

  public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA =
      new Transform3d(
          new Translation3d(.0178, -0.3072, 0.40311),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(-30.0), Math.toRadians(-163.0)));

  // verify pitch, should front be positive and back negative ?   or 30+180 ???? 210
  //
  public static final String FRONT_LEFT_CAMERA_NAME = "OV9281FrontLeft";
  public static final String FRONT_RIGHT_CAMERA_NAME = "OV9281FrontRight";
  public static final String BACK_LEFT_CAMERA_NAME = "OV9281BackLeft";
  public static final String BACK_RIGHT_CAMERA_NAME = "OV9281BackRight";

  public static final Vector<N3> STATE_STD_DEVS =
      VecBuilder.fill(.8, .8, Units.degreesToRadians(30));
  public static final Vector<N3> VISION_STD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
  public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS =
      MatBuilder.fill(Nat.N3(), Nat.N1(), 1, 1, 1 * Math.PI);

  public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
  public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
  public static final double TAG_PRESENCE_WEIGHT = 10;
  public static final double NOISY_DISTANCE_METERS = 3.2;
  public static final double DISTANCE_WEIGHT = 20;

  public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.09;
  public static final double APRILTAG_CULL_DISTANCE = 4.0;

  public static final String OBJECT_DETECTION_CAMERA = "OV9782NoteCamera";
  public static final String CAMERA2_NAME = "OV9281_2";

  public enum TAG {
    BLUE_SOURCE_RIGHT(1),
    BLUE_SOURCE_LEFT(2),
    RED_SPEAKER_RIGHT(3),
    RED_SPEAKER_CENTER(4),
    RED_AMP(5),
    BLUE_AMP(6),
    BLUE_SPEAKER_CENTER(7),
    BLUE_SPEAKER_LEFT(8),
    RED_SOURCE_RIGHT(9),
    RED_SOURCE_LEFT(10),
    RED_STAGE_LEFT(11),
    RED_STAGE_RIGHT(12),
    RED_STAGE_CENTER(13),
    BLUE_STAGE_CENTER(14),
    BLUE_STAGE_LEFT(15),
    BLUE_STAGE_RIGHT(16);

    public final int tagNumber;

    public int getValue() {
      return tagNumber;
    }

    private TAG(int tagNumber) {
      this.tagNumber = tagNumber;
    }
  }
}
