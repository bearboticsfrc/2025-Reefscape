// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import bearlib.util.ProcessedJoystick;
import bearlib.util.ProcessedJoystick.JoystickAxis;
import bearlib.util.ProcessedJoystick.ThrottleProfile;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BargeScoreCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.ReefScoreCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.field.ReefAutoAlignZones;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.CoralSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;
import java.util.ArrayList;

@Logged
public class RobotContainer {
  private final CommandXboxController driverJoystick =
      new CommandXboxController(DriveConstants.DRIVER_JOYSTICK_PORT);

  private final CommandXboxController operatorJoystick =
      new CommandXboxController(DriveConstants.OPERATOR_JOYSTICK_PORT);

  private final CommandGenericHID operatorGamepad =
      new CommandGenericHID(DriveConstants.OPERATOR_GAMEPAD_PORT);

  private final ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile, DriveConstants.MAX_VELOCITY);
  private ThrottleProfile throttleProfile = ThrottleProfile.TURBO;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CoralSubsystem coral = new CoralSubsystem();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

  @NotLogged private SendableChooser<Command> autoChooser;

  private ElevatorPosition targetElevatorPosition = ElevatorPosition.HOME;

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
    configureAutoBuilder();
  }

  /** Configure the button bindings. */
  private void configureDriverBindings() {
    driverJoystick.leftBumper().whileTrue(coral.intakeCoral()).onFalse(coral.stopIntake());

    driverJoystick.rightBumper().onTrue(coral.scoreCoral()).onFalse(coral.stopIntake());

    driverJoystick
        .rightTrigger()
        .whileTrue(
            elevator
                .runElevatorTo(() -> targetElevatorPosition)
                .andThen(Commands.waitUntil(elevator::isAtSetpoint)))
        .onFalse(elevator.runElevatorTo(ElevatorPosition.HOME));

    driverJoystick.b().whileTrue(algae.scoreAlgae()).onFalse(algae.stopMotor());
    driverJoystick
        .a()
        .whileTrue(arm.runArmTo(ArmPosition.REEF).andThen(algae.intakeAlgae()))
        .onFalse(arm.runArmTo(ArmPosition.HOME).andThen(algae.stopMotor()));

    driverJoystick.x().onTrue(Commands.runOnce(drivetrain.orchestra::play));
    driverJoystick.y().onTrue(Commands.runOnce(drivetrain.orchestra::stop));

    driverJoystick.leftTrigger().whileTrue(algae.intakeAlgae()).onFalse(algae.stopMotor());

    driverJoystick
        .leftStick()
        .toggleOnTrue(
            Commands.startEnd(
                () -> setThrottleProfile(ThrottleProfile.TURTLE),
                () -> setThrottleProfile(ThrottleProfile.TURBO)));

    driverJoystick
        .povUp()
        .whileTrue(BargeScoreCommand.raise(elevator, arm, algae))
        .whileFalse(BargeScoreCommand.lower(elevator, arm, algae));

    driverJoystick.povLeft().whileTrue(new DriveToPose(drivetrain, this::getNearestRightPose));
    driverJoystick.povRight().whileTrue(new DriveToPose(drivetrain, this::getNearestLeftPose));

    driverJoystick.povDown().whileTrue(coral.reverseCoral()).onFalse(coral.stopIntake());

    drivetrain.registerTelemetry(DriveConstants.TELEMETRY::telemeterize);
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));
  }

  private void configureOperatorBindings() {
    operatorGamepad
        .button(1)
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L1)));

    operatorGamepad
        .button(2)
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L2)));

    operatorGamepad
        .button(3)
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L3)));

    operatorGamepad
        .button(4)
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L4)));
  }

  public Pose2d getNearestLeftPose() {
    return ReefAutoAlignZones.tagPoses.get(getNearestTagPose()).left;
  }

  public Pose2d getNearestRightPose() {
    return ReefAutoAlignZones.tagPoses.get(getNearestTagPose()).right;
  }

  public Pose2d getNearestTagPose() {
    ArrayList<Pose2d> list = new ArrayList<>();
    list.addAll(ReefAutoAlignZones.tagPoses.keySet());

    return drivetrain.getState().Pose.nearest(list);
  }

  /**
   * Gets the default drive request using field centric mode.
   *
   * @return The default drive request.
   */
  private SwerveRequest getDefaultDriveRequest() {
    return DriveConstants.FIELD_CENTRIC_SWERVE_REQUEST
        .withVelocityX(processedJoystick.get(JoystickAxis.Ly))
        .withVelocityY(processedJoystick.get(JoystickAxis.Lx))
        .withRotationalRate(processedJoystick.get(JoystickAxis.Rx));
  }

  /** Configures the PathPlanner autobuilder for publishing on NT. */
  private void configureAutoBuilder() {
    NamedCommands.registerCommand(
        "L4Score", ReefScoreCommand.get(ElevatorPosition.L4, elevator, coral));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Path", autoChooser);
  }

  private void setTargetElevatorPosition(ElevatorPosition position) {
    targetElevatorPosition = position;
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

  /**
   * Get the currently selected autonomous command.
   *
   * @return The command.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
