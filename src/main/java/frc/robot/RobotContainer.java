// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import bearlib.util.ProcessedJoystick;
import bearlib.util.ProcessedJoystick.JoystickAxis;
import bearlib.util.ProcessedJoystick.ThrottleProfile;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveToPose;
import frc.robot.constants.DriveConstants;
import frc.robot.field.NearestReefZone;
import frc.robot.field.ReefAutoAlignZone;
import frc.robot.field.ReefAutoAlignZones;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.function.Supplier;

public class RobotContainer {
  private final NearestReefZone nearestReefZone = new NearestReefZone();
  private final CommandXboxController driverJoystick = new CommandXboxController(0);

  private final ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile, DriveConstants.MAX_VELOCITY);
  private ThrottleProfile throttleProfile = ThrottleProfile.NORMAL;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    configureAutoBuilder();
  }

  private void configureBindings() {
    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driverJoystick
        .rightStick()
        .onTrue(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.TURBO)))
        .onFalse(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.NORMAL)));

    driverJoystick
        .leftStick()
        .onTrue(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.TURTLE)))
        .onFalse(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.NORMAL)));

    //    driverJoystick.rightBumper().whileTrue(reefOrientedDriveRequestCommand());
    driverJoystick.rightBumper().whileTrue(driveToNearestBranchCommand());

    drivetrain.registerTelemetry(DriveConstants.TELEMETRY::telemeterize);
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));
  }

  public void periodic() {
    logZone();
  }

  private void logZone() {
    Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
    Optional<ReefAutoAlignZone> zone = ReefAutoAlignZones.inZone(robotPosition);
    double degrees = 0;
    if (zone.isPresent()) {
      degrees = zone.get().angle.getDegrees();
    }
    SmartDashboard.putNumber("Zone", degrees);
  }

  /**
   * Gets the default drive request using field centric mode..
   *
   * @return The default drive request.
   */
  private SwerveRequest getDefaultDriveRequest() {
    return DriveConstants.FIELD_CENTRIC_SWERVE_REQUEST
        .withVelocityX(processedJoystick.get(JoystickAxis.Ly))
        .withVelocityY(processedJoystick.get(JoystickAxis.Lx))
        .withRotationalRate(processedJoystick.get(JoystickAxis.Rx));
  }

  private double getReefOrientationDegrees() {
    Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
    Optional<ReefAutoAlignZone> zone = ReefAutoAlignZones.inZone(robotPosition);
    double degrees = 0;
    if (zone.isPresent()) {
      degrees = zone.get().angle.getDegrees();
    }
    return degrees;
  }

  private Command reefOrientedDriveRequestCommand() {
    return drivetrain.applyRequest(
        () ->
            DriveConstants.REEF_ORIENTED_SWERVE_REQUEST
                .withVelocityX(processedJoystick.get(JoystickAxis.Ly))
                .withVelocityY(processedJoystick.get(JoystickAxis.Lx))
                .withTargetDirection(Rotation2d.fromDegrees(getReefOrientationDegrees())));
  }

  private Command driveToNearestBranchCommand() {
    Supplier<Pose2d> poseSupplier =
        () -> nearestReefZone.getNearestBranch(drivetrain.getState().Pose);
    return new DriveToPose(drivetrain, poseSupplier);
  }

  private void configureAutoBuilder() {
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Sets the throttle profile for the robot.
   *
   * @param throttleProfile An enum representing the throttle profile.
   */
  private void setThrottleProfile(ThrottleProfile throttleProfile) {
    this.throttleProfile = throttleProfile;
  }

  /**
   * Get the current throttle profile for the robot.
   *
   * @return The throttle profile
   */
  private ThrottleProfile getThrottleProfile() {
    return throttleProfile;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
