package frc.robot.commands;

// import static frc.robot.constants.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceFlipUtil;

public class AutoBargeAlignCommand extends Command {
  private static final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(6);
  private static final LinearAcceleration TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(4);

  private static final Pose2d BLUE_SCORE_POSE = new Pose2d(8.092, 0, Rotation2d.k180deg);

  public static Command get(CommandSwerveDrivetrain drivetrain) {
    return new DriveToPoseCommand(
            drivetrain, () -> getTargetPose(drivetrain.getState().Pose, drivetrain))
        .withTranslationConstraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION);
  }

  private static Pose2d getTargetPose(Pose2d currentPose, CommandSwerveDrivetrain drivetrain) {
    final Pose2d pose =
        AllianceFlipUtil.apply(
            BLUE_SCORE_POSE.plus(new Transform2d(0, currentPose.getY(), Rotation2d.kZero)));
    drivetrain.bargeAlignPose = pose;
    return pose;
  }
}
