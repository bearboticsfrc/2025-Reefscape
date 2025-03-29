package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
// import static frc.robot.constants.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceFlipUtil;

public class AutoBargeAlignCommand extends Command {
  private static final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(4);
  private static final LinearAcceleration TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(4);

  private static final Distance TRANSLATION_TOLERANCE = Centimeters.of(2);
  private static final Angle HEADING_TOLERANCE = Degrees.of(3);

  private static final Pose2d BLUE_SCORE_POSE = new Pose2d(8.092, 0, Rotation2d.k180deg);

  public static Command get(CommandSwerveDrivetrain drivetrain) {
    return new DriveToPoseCommand(
            drivetrain, () -> getTargetPose(drivetrain.getState().Pose, drivetrain))
        .withTranslationPID(7, 0, 0)
        .withTranslationConstraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION)
        .withTranslationTolerance(TRANSLATION_TOLERANCE)
        .withHeadingTolerance(HEADING_TOLERANCE)
        .withDebounceDuration(Seconds.of(0.2));
  }

  private static Pose2d getTargetPose(Pose2d currentPose, CommandSwerveDrivetrain drivetrain) {
    final Pose2d flippedPose = AllianceFlipUtil.apply(BLUE_SCORE_POSE);
    final Pose2d bargePose =
        new Pose2d(flippedPose.getX(), currentPose.getY(), flippedPose.getRotation());

    drivetrain.bargeAlignPose = bargePose;
    return bargePose;
  }
}
