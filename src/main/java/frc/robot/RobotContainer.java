// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import bearlib.util.ProcessedJoystick;
import bearlib.util.ProcessedJoystick.JoystickAxis;
import bearlib.util.ProcessedJoystick.ThrottleProfile;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.CoralSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;

@Logged
public class RobotContainer {
  private final CommandXboxController driverJoystick = new CommandXboxController(0);

  private final ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile, DriveConstants.MAX_VELOCITY);
  private ThrottleProfile throttleProfile = ThrottleProfile.NORMAL;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CoralSubsystem coral = new CoralSubsystem();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverJoystick.leftBumper().whileTrue(coral.intakeCoral()).onFalse(coral.stopIntake());
    driverJoystick.rightBumper().whileTrue(coral.scoreCoral()).onFalse(coral.stopIntake());

    driverJoystick.povRight().onTrue(arm.runArmTo(ArmPosition.REEF));
    driverJoystick.povDown().onTrue(arm.runArmTo(ArmPosition.HOME));
    driverJoystick.povUp().onTrue(algae.scoreAlgae()).onFalse(algae.stopMotor());

    driverJoystick.leftTrigger().whileTrue(algae.intakeAlgae()).onFalse(algae.stopMotor());
    driverJoystick
        .rightTrigger()
        .whileTrue(
            elevator
                .runElevatorTo(ElevatorPosition.L4)
                .andThen(Commands.waitUntil(elevator::isAtSetpoint))
                .andThen(arm.runArmTo(ArmPosition.BARGE))
                .andThen(Commands.waitUntil(arm::isAtSetpoint))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(algae.scoreAlgae())
                .andThen(arm.runArmTo(ArmPosition.HOME))
                .andThen(Commands.waitUntil(arm::isAtSetpoint))
                .andThen(elevator.runElevatorTo(ElevatorPosition.HOME))
                .andThen(Commands.waitUntil(elevator::isAtSetpoint)))
        .whileFalse(
            arm.runArmTo(ArmPosition.HOME)
                .andThen(Commands.waitUntil(arm::isAtSetpoint))
                .andThen(elevator.runElevatorTo(ElevatorPosition.HOME))
                .andThen(Commands.waitUntil(elevator::isAtSetpoint))
                .alongWith(algae.stopMotor()));

    driverJoystick.y().onTrue(elevator.runElevatorTo(ElevatorPosition.L4));
    driverJoystick.x().onTrue(elevator.runElevatorTo(ElevatorPosition.L3));
    driverJoystick.b().onTrue(elevator.runElevatorTo(ElevatorPosition.L2));
    driverJoystick.a().onTrue(elevator.runElevatorTo(ElevatorPosition.HOME));

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
