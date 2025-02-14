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

import bearlib.fms.AllianceColor;
import bearlib.util.TunableNumber;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.TunableNumberUtil;
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
  private final double LOOP_PERIOD_SECS = 0.02;
  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose;
  private Transform2d targetTolerance;

  private boolean running = false;
  private Timer timer;

  private static final TunableNumber driveKp = new TunableNumber("DriveToPose/DriveKp", 1.0);
  private static final TunableNumber driveKd = new TunableNumber("DriveToPose/DriveKd", 0.0);
  private static final TunableNumber driveKi = new TunableNumber("DriveToPose/DriveKi", 0);
  private static final TunableNumber thetaKp = new TunableNumber("DriveToPose/ThetaKp", 10.0);
  private static final TunableNumber thetaKd = new TunableNumber("DriveToPose/ThetaKd", 0.0);
  private static final TunableNumber thetaKi = new TunableNumber("DriveToPose/ThetaKi", 0.0);
  private static final TunableNumber driveMaxVelocity =
      new TunableNumber(
          "DriveToPose/DriveMaxVelocityMetersPerSecond",
          MetersPerSecond.of(3.0).in(MetersPerSecond));
  private static final TunableNumber driveMaxAcceleration =
      new TunableNumber(
          "DriveToPose/DriveMaxAccelerationMetersPerSecondPerSecond",
          MetersPerSecondPerSecond.of(6.0).in(MetersPerSecondPerSecond));
  private static final TunableNumber thetaMaxVelocity =
      new TunableNumber(
          "DriveToPose/ThetaMaxVelocityRadiansPerSecond",
          RadiansPerSecond.of(2 * Math.PI).in(RadiansPerSecond));
  private static final TunableNumber thetaMaxAcceleration =
      new TunableNumber(
          "DriveToPose/ThetaMaxAccelerationRadiansPerSecondPerSecond",
          RadiansPerSecondPerSecond.of(2 * Math.PI).in(RadiansPerSecondPerSecond));
  private static final TunableNumber driveTolerance =
      new TunableNumber("DriveToPose/DriveToleranceMeters", Meters.of(0.01).in(Meters));
  private static final TunableNumber thetaTolerance =
      new TunableNumber("DriveToPose/ThetaToleranceRadians", Radians.of(0.017).in(Radians));
  private static final TunableNumber timeout = new TunableNumber("DriveToPose/timeout", 5.0);

  private StructPublisher<Pose3d> scorePosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("ScorePose", Pose3d.struct).publish();

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          driveKp.get(),
          driveKi.get(),
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          driveKp.get(),
          driveKi.get(),
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
          LOOP_PERIOD_SECS);

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToPose(
      CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> poseSupplier, Transform2d tolerance) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.targetTolerance = tolerance;
    this.timer = new Timer();
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
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
    // Reset all controllers
    Pose2d currentPose = drivetrain.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
    xController.setTolerance(driveTolerance.get());
    yController.setTolerance(driveTolerance.get());
    thetaController.setTolerance(thetaTolerance.get());
    this.targetPose = poseSupplier.get();
    scorePosePublisher.set(new Pose3d(targetPose));

    // drivetrain.enableAccelerationLimiting();
    // Logger.recordOutput("DriveToPose/targetPose", targetPose);

    this.timer.restart();
    System.out.println("Drive to pose started for " + targetPose.toString());
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    // set running to true in this method to capture that the calculate method has been invoked on
    // the PID controllers. This is important since these controllers will return true for atGoal if
    // the calculate method has not yet been invoked.
    running = true;

    // Update from tunable numbers
    TunableNumberUtil.ifChanged(
        hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
          yController.setPID(pid[0], pid[1], pid[2]);
        },
        driveKp,
        driveKi,
        driveKd);
    TunableNumberUtil.ifChanged(
        hashCode(),
        max -> {
          xController.setConstraints(new TrapezoidProfile.Constraints(max[0], max[1]));
          yController.setConstraints(new TrapezoidProfile.Constraints(max[0], max[1]));
        },
        driveMaxVelocity,
        driveMaxAcceleration);
    TunableNumberUtil.ifChanged(
        hashCode(),
        tolerance -> {
          xController.setTolerance(tolerance[0]);
          yController.setTolerance(tolerance[0]);
        },
        driveTolerance);

    TunableNumberUtil.ifChanged(
        hashCode(),
        pid -> thetaController.setPID(pid[0], pid[1], pid[2]),
        thetaKp,
        thetaKi,
        thetaKd);
    TunableNumberUtil.ifChanged(
        hashCode(),
        max -> thetaController.setConstraints(new TrapezoidProfile.Constraints(max[0], max[1])),
        thetaMaxVelocity,
        thetaMaxAcceleration);
    TunableNumberUtil.ifChanged(
        hashCode(), tolerance -> thetaController.setTolerance(tolerance[0]), thetaTolerance);

    Pose2d currentPose = drivetrain.getPose();

    // Transform2d difference = currentPose.minus(targetPose);
    // double highVelocityDistanceThresholdMeters = 1.0; // arbitrary 1 meters away right now
    // double straightLineHighVelocityMPS = 2.0; // arbitrary 2 m/s right now

    // use last values of filter
    final double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
    final double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());
    final double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());

    // xVelocity =
    //     Math.abs(difference.getX()) > highVelocityDistanceThresholdMeters
    //         ? (xController.calculate(currentPose.getX(), this.targetPose.getX()) >= 0
    //             ? straightLineHighVelocityMPS
    //             : -straightLineHighVelocityMPS)
    //         : xController.calculate(currentPose.getX(), this.targetPose.getX());
    // yVelocity =
    //     Math.abs(difference.getY()) > highVelocityDistanceThresholdMeters
    //         ? (yController.calculate(currentPose.getY(), this.targetPose.getY()) >= 0
    //             ? straightLineHighVelocityMPS
    //             : -straightLineHighVelocityMPS)
    //         : yController.calculate(currentPose.getY(), this.targetPose.getY());
    // thetaVelocity =
    //     thetaController.calculate(
    //         currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());

    // if (xController.atGoal()) xVelocity = 0.0;
    // if (yController.atGoal()) yVelocity = 0.0;
    // if (thetaController.atGoal()) thetaVelocity = 0.0;

    int allianceMultiplier = AllianceColor.getAlliance() == Alliance.Blue ? 1 : -1;

    System.out.println("X[" + xVelocity + "] Y[" + yVelocity + "] theta[" + thetaVelocity + "]");
    // drivetrain.drive(
    //     allianceMultiplier * xVelocity, allianceMultiplier * yVelocity, thetaVelocity, true,
    // true);
    drivetrain.setControl(
        DriveConstants.DRIVE_TO_POSE_SWERVE_REQUEST
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
    Transform2d difference = drivetrain.getPose().minus(targetPose);
    // Logger.recordOutput("DriveToPose/difference", difference);

    boolean atGoal =
        Math.abs(difference.getX()) < targetTolerance.getX()
            && Math.abs(difference.getY()) < targetTolerance.getY()
            && Math.abs(difference.getRotation().getRadians())
                < targetTolerance.getRotation().getRadians();

    // check that running is true (i.e., the calculate method has been invoked on the PID
    // controllers) and that each of the controllers is at their goal. This is important since these
    // controllers will return true for atGoal if the calculate method has not yet been invoked.
    //    return this.timer.hasElapsed(timeout.get()) || atGoal;
    return atGoal;
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    // drivetrain.disableAccelerationLimiting();
    drivetrain.stop();
    running = false;
    System.out.println("Drive to pose stopped for " + targetPose.toString());
  }
}
