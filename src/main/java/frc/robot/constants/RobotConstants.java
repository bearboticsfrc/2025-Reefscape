package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Constants related to the robot's physical dimensions, components, and control system. */
public class RobotConstants {
  // Robot dimensions in meters
  public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
  public static final double WHEEL_BASE = Units.inchesToMeters(18.75);
  public static final double WHEEL_DIAMETER = 0.097;
  public static final double SWERVE_RADIUS = 0.3552;
  public static final double CENTER_TO_BUMPER = .55;

  // Shuffleboard tabs
  public static final ShuffleboardTab SHOOTER_SYSTEM_TAB = Shuffleboard.getTab("Shooter System");
  public static final ShuffleboardTab ARM_SYSTEM_TAB = Shuffleboard.getTab("Arm System");
  public static final ShuffleboardTab CLIMBER_SYSTEM_TAB = Shuffleboard.getTab("Climber System");
  public static final ShuffleboardTab INTAKE_SYSTEM_TAB = Shuffleboard.getTab("Intake System");
  public static final ShuffleboardTab DRIVE_SYSTEM_TAB = Shuffleboard.getTab("Drive System");
  public static final ShuffleboardTab VISION_SYSTEM_TAB = Shuffleboard.getTab("VIsion System");

  public static final ShuffleboardTab COMPETITION_TAB = Shuffleboard.getTab("Competition");
  public static final ShuffleboardTab TEST_TAB = Shuffleboard.getTab("Test");

  public static final ShuffleboardTab TUNING_TAB = Shuffleboard.getTab("Tuning");

  // Pigeon IMU CAN ID
  public static final int PIGEON_CAN_ID = 20;

  // Control cycle time in seconds
  public static final double CYCLE_TIME = Units.millisecondsToSeconds(20);

  // Swerve drive kinematics
  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Back Left
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Back Right

  // Enumeration for SwerveDriveKinematics corner locations
  public enum SwerveCorner {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
  }
}
