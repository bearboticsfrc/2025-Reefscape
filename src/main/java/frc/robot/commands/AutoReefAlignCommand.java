// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.reef.ReefTagPoses;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoReefAlignCommand extends Command {
  private static final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(12);
  private static final LinearAcceleration TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(6);

  private static final Distance TRANSLATION_TOLERANCE = Centimeters.of(3);
  private static final Angle HEADING_TOLERANCE = Degrees.of(1);

  public static Command get(CommandSwerveDrivetrain drivetrain, ReefTagPoses.ScoreSide side) {
    return new DriveToPoseCommand(drivetrain, () -> getTargetPose(drivetrain, side))
        .withTranslationConstraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION)
        .withTranslationPID(2.5, 0, 0)
        .withDebounceDuration(Seconds.of(0))
        .withTranslationTolerance(TRANSLATION_TOLERANCE)
        .withThetaTolerance(HEADING_TOLERANCE);
  }

  public static Pose2d getTargetPose(
      CommandSwerveDrivetrain drivetrain, ReefTagPoses.ScoreSide side) {
    final Pose2d pose = ReefTagPoses.getNearestScoringPose(drivetrain.getState().Pose, side);
    drivetrain.reefAlignPose = pose;
    return pose;
  }
}
