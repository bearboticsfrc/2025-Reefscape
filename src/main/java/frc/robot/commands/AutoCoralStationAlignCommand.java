// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.VisionConstants.CORAL_STATION_TAGS_ONLY_LAYOUT;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutoCoralStationAlignCommand extends Command {
  private static Optional<List<Pose2d>> maybePoses = Optional.empty();

  private static final Distance CENTER_TO_CORAL_STATION = Inches.of(16);
  private static final Transform2d POSE_TRANSFORM =
      new Transform2d(new Translation2d(CENTER_TO_CORAL_STATION, Inches.of(0)), Rotation2d.kZero);

  private static final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(12);
  private static final LinearAcceleration TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(6);

  private static final Distance TRANSLATION_TOLERANCE = Centimeters.of(0.25);
  private static final Angle HEADING_TOLERANCE = Degrees.of(0.5);

  public static Command get(CommandSwerveDrivetrain drivetrain) {
    return new DriveToPoseCommand(
            drivetrain, () -> drivetrain.getState().Pose.nearest(getCoralStationPoses()))
        .withTranslationConstraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION)
        .withTranslationPID(5, 0, 0)
        .withDebounceDuration(Seconds.of(0))
        .withTranslationTolerance(TRANSLATION_TOLERANCE)
        .withHeadingTolerance(HEADING_TOLERANCE);
  }

  public static List<Pose2d> getCoralStationPoses() {
    if (maybePoses.isPresent()) {
      return maybePoses.get();
    }

    List<Pose2d> tagPoses = new ArrayList<Pose2d>();

    for (AprilTag tag : CORAL_STATION_TAGS_ONLY_LAYOUT.getTags()) {
      tagPoses.add(tag.pose.toPose2d().plus(POSE_TRANSFORM));
    }

    maybePoses = Optional.of(tagPoses);
    return tagPoses;
  }
}
