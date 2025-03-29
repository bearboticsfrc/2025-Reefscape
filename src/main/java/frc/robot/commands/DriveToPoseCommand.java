// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line using profiled PID controllers for translation and a standard PID controller for
 * heading.
 *
 * <p>This command is intended for short, precise movements during teleoperated control or
 * autonomous routines where a simple straight path is sufficient.
 *
 * <p>Requires: the Drivetrain subsystem.
 *
 * <p>Finished When: the robot is at the specified pose (within the defined tolerances of the
 * controllers) for a duration defined by the finish debouncer.
 *
 * <p>At End: stops the drivetrain.
 */
public class DriveToPoseCommand extends Command {

  // --- Constants for Default Behavior ---
  private static final double DEFAULT_TRANSLATION_P = 10.0;
  private static final double DEFAULT_TRANSLATION_I = 0.0;
  private static final double DEFAULT_TRANSLATION_D = 0.0; // Added D for completeness, though 0

  private static final double DEFAULT_THETA_P = 10.0;
  private static final double DEFAULT_THETA_I = 0.0;
  private static final double DEFAULT_THETA_D = 0.0;

  private static final LinearVelocity DEFAULT_MAX_LINEAR_VELOCITY = MetersPerSecond.of(8.0);
  private static final LinearAcceleration DEFAULT_MAX_LINEAR_ACCELERATION =
      MetersPerSecondPerSecond.of(5.0);

  private static final TrapezoidProfile.Constraints DEFAULT_TRANSLATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          DEFAULT_MAX_LINEAR_VELOCITY.in(MetersPerSecond),
          DEFAULT_MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond));

  private static final Time IS_FINISHED_DEBOUNCE_TIME = Seconds.of(0.2);

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;

  private final SwerveRequest.FieldCentricFacingAngle driveRequest =
      new FieldCentricFacingAngle().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private final ProfiledPIDController xTranslationController;
  private final ProfiledPIDController yTranslationController;

  private final Debouncer isFinishedDebouncer =
      new Debouncer(IS_FINISHED_DEBOUNCE_TIME.in(Seconds));

  private Pose2d targetPose;

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created. Uses default PID and constraint values.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the target pose to drive to
   */
  public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;

    this.xTranslationController =
        new ProfiledPIDController(
            DEFAULT_TRANSLATION_P,
            DEFAULT_TRANSLATION_I,
            DEFAULT_TRANSLATION_D,
            DEFAULT_TRANSLATION_CONSTRAINTS);
    this.yTranslationController =
        new ProfiledPIDController(
            DEFAULT_TRANSLATION_P,
            DEFAULT_TRANSLATION_I,
            DEFAULT_TRANSLATION_D,
            DEFAULT_TRANSLATION_CONSTRAINTS);

    this.xTranslationController.setTolerance(0);
    this.yTranslationController.setTolerance(0);

    this.driveRequest.HeadingController.setPID(DEFAULT_THETA_P, DEFAULT_THETA_I, DEFAULT_THETA_D);
    this.driveRequest.HeadingController.setTolerance(0);

    addRequirements(drivetrain);
  }

  /**
   * This method is invoked once when this command is scheduled. It resets all the PID controllers,
   * retrieves the target pose from the supplier, and initializes the controllers with the current
   * and target poses. It also resets the finish debouncer.
   */
  @Override
  public void initialize() {
    final Pose2d currentPose = drivetrain.getState().Pose;
    targetPose = poseSupplier.get(); // Get the latest target pose

    // Reset and configure translation controllers
    xTranslationController.reset(currentPose.getX());
    xTranslationController.setGoal(targetPose.getX());

    yTranslationController.reset(currentPose.getY());
    yTranslationController.setGoal(targetPose.getY());

    // Reset and configure heading controller (part of the SwerveRequest)
    // Current heading is implicitly handled by the Swerve Drivetrain/Request system
    driveRequest.HeadingController.reset();

    // Reset the debouncer
    isFinishedDebouncer.calculate(false);
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the required
   * field-relative X and Y velocities based on the profiled PID controllers and the current robot
   * pose. It then instructs the drivetrain to drive using these velocities and to face the target
   * pose's rotation.
   */
  @Override
  public void execute() {
    final Pose2d currentPose = drivetrain.getState().Pose;

    // Calculate translational velocities using profiled PID
    final double xVelocityOutput = xTranslationController.calculate(currentPose.getX());
    final double yVelocityOutput = yTranslationController.calculate(currentPose.getY());

    // Update the swerve request and send it to the drivetrain
    // The heading controller inside driveRequest calculates its own output based on the target
    drivetrain.setControl(
        driveRequest
            .withVelocityX(xVelocityOutput) // Field-relative X velocity
            .withVelocityY(yVelocityOutput) // Field-relative Y velocity
            .withTargetDirection(targetPose.getRotation())); // Target orientation
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked).
   *
   * @return true if the robot has reached the target translational position and heading orientation
   *     (within controller tolerances) for the debouncer duration.
   */
  @Override
  public boolean isFinished() {
    // Check if controllers are at their goal/setpoint
    boolean translationReached = xTranslationController.atGoal() && yTranslationController.atGoal();
    boolean rotationReached = driveRequest.HeadingController.atSetpoint();

    // Use the debouncer to ensure the robot stays at the target pose
    return isFinishedDebouncer.calculate(translationReached && rotationReached);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain by sending an idle request.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled.
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }

  /**
   * Sets the PID constants for the translation controllers.
   *
   * @param p Proportional gain
   * @param i Integral gain
   * @param d Derivative gain
   * @return This command instance for chaining.
   */
  public DriveToPoseCommand withTranslationPID(double p, double i, double d) {
    xTranslationController.setPID(p, i, d);
    yTranslationController.setPID(p, i, d);
    return this;
  }

  /**
   * Sets the PID constants for the heading controller.
   *
   * @param p Proportional gain
   * @param i Integral gain
   * @param d Derivative gain
   * @return This command instance for chaining.
   */
  public DriveToPoseCommand withHeadingPID(double p, double i, double d) {
    driveRequest.HeadingController.setPID(p, i, d);

    return this;
  }

  /**
   * Sets the motion profile constraints for the translation controllers.
   *
   * @param maxLinearVelocity Maximum linear velocity.
   * @param maxLinearAcceleration Maximum linear acceleration.
   * @return This command instance for chaining.
   */
  public DriveToPoseCommand withTranslationConstraints(
      LinearVelocity maxLinearVelocity, LinearAcceleration maxLinearAcceleration) {
    final TrapezoidProfile.Constraints translationConstraints =
        new TrapezoidProfile.Constraints(
            maxLinearVelocity.in(MetersPerSecond),
            maxLinearAcceleration.in(MetersPerSecondPerSecond));

    xTranslationController.setConstraints(translationConstraints);
    yTranslationController.setConstraints(translationConstraints);

    return this;
  }

  public DriveToPoseCommand withTranslationTolerance(Distance tolerance) {
    this.xTranslationController.setTolerance(tolerance.in(Meters));
    this.yTranslationController.setTolerance(tolerance.in(Meters));

    return this;
  }

  public DriveToPoseCommand withThetaTolerance(Angle tolerance) {
    this.driveRequest.HeadingController.setTolerance(tolerance.in(Rotations));

    return this;
  }

  public DriveToPoseCommand withDebounceDuration(Time duration) {
    this.isFinishedDebouncer.setDebounceTime(duration.in(Seconds));

    return this;
  }
}
