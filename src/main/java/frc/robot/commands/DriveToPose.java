// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
// import static frc.robot.constants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

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
public class DriveToPose extends Command {
  private final double TRANSLATION_K = 6;
  private final double THETA_K = 10;

  private final double TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(3).in(MetersPerSecond);
  private final double TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(6.0).in(MetersPerSecondPerSecond);

  private final double THETA_MAX_VELOCITY = RadiansPerSecond.of(2 * Math.PI).in(RadiansPerSecond);
  private final double THETA_MAX_ACCELERATION =
      RadiansPerSecondPerSecond.of(2 * Math.PI).in(RadiansPerSecondPerSecond);

  private final double POSITION_TOLERANCE = Inches.of(3).in(Meters);
  private final double ROTATIONS_TOLERANCE = Radians.of(0.017).in(Radians);

  private final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION);
  private final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(THETA_MAX_VELOCITY, THETA_MAX_ACCELERATION);

  private final SwerveRequest.FieldCentric DRIVE_TO_POSE = new FieldCentric();

  private final ProfiledPIDController xController =
      new ProfiledPIDController(TRANSLATION_K, 0, 0, TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(TRANSLATION_K, 0, 0, TRANSLATION_CONSTRAINTS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(THETA_K, 0, 0, THETA_CONSTRAINTS);

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;

  private Pose2d targetPose;

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToPose(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(ROTATIONS_TOLERANCE);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
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
    targetPose = poseSupplier.get();

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());

    Pose2d currentPose = drivetrain.getPose();

    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    final double xVelocity = xController.calculate(currentPose.getX());
    final double yVelocity = yController.calculate(currentPose.getY());
    final double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians());

    drivetrain.setControl(
        DRIVE_TO_POSE
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(thetaVelocity));
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
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
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
