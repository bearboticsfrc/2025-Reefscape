package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.reef.ReefTagPoses;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoReefAlignCommand extends Command {

  private static final Distance TRANSLATION_TOLERANCE = Centimeters.of(3);
  private static final Angle HEADING_TOLERANCE = Degrees.of(2);

  public static Command get(CommandSwerveDrivetrain drivetrain, ReefTagPoses.ScoreSide side) {
    return new DriveToPoseCommand(drivetrain, () -> getTargetPose(drivetrain, side))
        .withTranslationPID(4, 0, 0)
        .withDebounceDuration(Seconds.of(0))
        .withTranslationTolerance(TRANSLATION_TOLERANCE)
        .withHeadingTolerance(HEADING_TOLERANCE);
  }

  public static Pose2d getTargetPose(
      CommandSwerveDrivetrain drivetrain, ReefTagPoses.ScoreSide side) {
    final Pose2d pose = ReefTagPoses.getNearestScoringPose(drivetrain.getState().Pose, side);
    drivetrain.reefAlignPose = pose;
    return pose;
  }
}
