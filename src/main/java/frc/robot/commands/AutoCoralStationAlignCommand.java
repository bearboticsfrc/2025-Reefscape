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

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class AutoCoralStationAlignCommand extends Command {
  private final double TRANSLATION_K = 10;
  private final double TRANSLATION_I = 0;

  private final double THETA_K = 10;

  private final double TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(8).in(MetersPerSecond);
  private final double TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(5).in(MetersPerSecondPerSecond);

  private final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION);

  private final SwerveRequest.FieldCentricFacingAngle DRIVE_TO_POSE = new FieldCentricFacingAngle();

  private final ProfiledPIDController xController =
      new ProfiledPIDController(TRANSLATION_K, TRANSLATION_I, 0, TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(TRANSLATION_K, TRANSLATION_I, 0, TRANSLATION_CONSTRAINTS);

  private final CommandSwerveDrivetrain drivetrain;

  @Logged(name = "Coral Station Auto Align Target Pose", importance = Importance.CRITICAL)
  private Pose2d targetPose;

  private static Optional<List<Pose2d>> maybePoses = Optional.empty();

  private static final Distance CENTER_TO_CORAL_STATION = Inches.of(16);

  private static final Transform2d POSE_TRANSFORM =
      new Transform2d(new Translation2d(CENTER_TO_CORAL_STATION, Inches.zero()), Rotation2d.kZero);

  private static final Debouncer IS_FINISHED_DEBOUNCER = new Debouncer(0.5);

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

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public AutoCoralStationAlignCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    DRIVE_TO_POSE.HeadingController.setP(THETA_K);

    addRequirements(drivetrain);
  }

  @Logged(importance = Importance.CRITICAL)
  public Pose2d getTargetPose() {
    return drivetrain.getState().Pose.nearest(getCoralStationPoses());
  }

  /**
   * This method is invoked once when this command is scheduled. It resets all the PID controllers
   * and initializes the current and target poses. It is critical that this initialization occurs in
   * this method and not the constructor as this object is constructed well before the command is
   * scheduled and the robot's pose will definitely have changed and the target pose may not be
   * known until this command is scheduled.
   */
  @Override
  public void initialize() {
    targetPose = new Pose2d(14.03, 5.87, Rotation2d.fromDegrees(-125));

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());

    Pose2d currentPose = drivetrain.getState().Pose;

    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    final double xVelocity = xController.calculate(currentPose.getX());
    final double yVelocity = yController.calculate(currentPose.getY());

    drivetrain.setControl(
        DRIVE_TO_POSE
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withTargetDirection(targetPose.getRotation()));
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). This command is considered finished if the
   * move-to-pose feature is disabled on the drivetrain subsystem or if the timeout has elapsed or
   * if all the PID controllers are at their goal.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    return IS_FINISHED_DEBOUNCER.calculate(
        xController.atGoal()
            && yController.atGoal()
            && DRIVE_TO_POSE.HeadingController.atSetpoint());
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
