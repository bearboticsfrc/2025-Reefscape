package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceFlipUtil;

/**
 * A command to drive the robot to align with the barge scoring position. The target X and Rotation
 * are fixed (relative to alliance), while the target Y is based on the robot's current Y position
 * when the command starts.
 */
public class AutoBargeAlignCommand extends DriveToPoseCommand {

  private static final double TRANSLATION_P = 7;
  private static final double TRANSLATION_D = 0;
  private static final double TRANSLATION_I = 0;

  private static final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(4);
  private static final LinearAcceleration TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(4);

  private static final Distance TRANSLATION_TOLERANCE = Centimeters.of(2);
  private static final Angle HEADING_TOLERANCE = Degrees.of(3);

  // Define the target pose for the Blue alliance side
  private static final Pose2d BLUE_SCORE_POSE = new Pose2d(8.092, 0, Rotation2d.fromDegrees(180));

  private final CommandSwerveDrivetrain drivetrain;

  /**
   * Creates a new AutoBargeAlignCommand. This command drives the robot to a specific X coordinate
   * and rotation suitable for scoring on the barge, while maintaining the robot's current Y
   * coordinate at the start of the command.
   *
   * @param drivetrain The drivetrain subsystem used for accessing robot pose and controlling
   *     movement.
   */
  public AutoBargeAlignCommand(CommandSwerveDrivetrain drivetrain) {
    super(drivetrain, () -> drivetrain.getState().Pose); // Temporary supplier, overridden below
    this.drivetrain = drivetrain;

    // Configure DriveToPoseCommand settings
    this.withTranslationPID(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D)
        .withTranslationConstraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION)
        .withTranslationTolerance(TRANSLATION_TOLERANCE)
        .withHeadingTolerance(HEADING_TOLERANCE)
        .withPoseSupplier(
            this::getTargetPose); // Override the pose supplier to use our dynamic target
  }

  /**
   * Calculates the target pose for barge alignment. This method retrieves the robot's current pose,
   * determines the alliance-specific target X and Rotation based on {@link #BLUE_SCORE_POSE}, and
   * combines these with the robot's current Y coordinate to create the final target pose.
   *
   * @return The calculated {@link Pose2d} representing the desired alignment position and
   *     orientation for scoring on the barge.
   */
  public Pose2d getTargetPose() {
    final Pose2d currentPose = drivetrain.getState().Pose;
    // Flip the blue target pose if we are on the red alliance
    final Pose2d allianceTargetPose = AllianceFlipUtil.apply(BLUE_SCORE_POSE);

    // Create the final target pose using the alliance-specific X and Rotation,
    // but keep the robot's current Y position.
    return new Pose2d(
        allianceTargetPose.getX(), currentPose.getY(), allianceTargetPose.getRotation());
  }
}
