// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import bearlib.util.ProcessedJoystick;
import bearlib.util.ProcessedJoystick.JoystickAxis;
import bearlib.util.ProcessedJoystick.ThrottleProfile;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private final CommandXboxController driverJoystick = new CommandXboxController(0);

  private final ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile, DriveConstants.MAX_VELOCITY);
  private ThrottleProfile throttleProfile = ThrottleProfile.NORMAL;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    driverJoystick
        .rightStick()
        .onTrue(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.TURBO)))
        .onFalse(Commands.runOnce(() -> setThrottleProfile(ThrottleProfile.NORMAL)));

    drivetrain.registerTelemetry(DriveConstants.TELEMETRY::telemeterize);
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));
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
    return Commands.print("No autonomous command configured");
  }
}
