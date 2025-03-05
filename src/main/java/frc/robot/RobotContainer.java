// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ProcessedJoystick.JoystickAxis;
import frc.robot.ProcessedJoystick.ThrottleProfile;
import frc.robot.commands.AutoCoralStationAlign;
import frc.robot.commands.AutoReefAlignCommand;
import frc.robot.commands.BargeScoreCommand;
import frc.robot.commands.ReefScoreCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.reef.ReefTagPoses;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.CoralSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utils.AllianceFlipUtil;
import java.lang.reflect.Method;

public class RobotContainer {
  private final CommandPS4Controller driverJoystick =
      new CommandPS4Controller(DriveConstants.DRIVER_JOYSTICK_PORT);

  private final CommandGenericHID operatorGamepad =
      new CommandGenericHID(DriveConstants.OPERATOR_GAMEPAD_PORT);

  private final ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile, DriveConstants.MAX_VELOCITY);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // TODO: update g-force value
  private Trigger trigger =
      new Trigger(() -> drivetrain.getPigeon2().getGravityVectorX().getValue() >= 0.9);

  @Logged private final CoralSubsystem coral = new CoralSubsystem();
  @Logged private final AlgaeSubsystem algae = new AlgaeSubsystem();
  @Logged private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  @Logged private final ArmSubsystem arm = new ArmSubsystem();

  private ProcessedJoystick.ThrottleProfile throttleProfile =
      ProcessedJoystick.ThrottleProfile.TURBO;

  @Logged(name = "Auto Chooser")
  private SendableChooser<Command> autoChooser;

  @Logged(name = "Auto Start Pose")
  private Pose2d autoStartPose;

  private Command introspectedAutoCommand;

  @Logged(name = "Target Elevator Position")
  private ElevatorPosition targetElevatorPosition = ElevatorPosition.HOME;

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
    configureAutoBuilder();
  }

  /** Configure the button bindings. */
  private void configureDriverBindings() {
    driverJoystick
        .L1()
        .whileTrue(
            new AutoCoralStationAlign(
                    drivetrain,
                    () -> processedJoystick.get(JoystickAxis.Ly),
                    () -> processedJoystick.get(JoystickAxis.Lx))
                .alongWith(coral.intakeCoral()))
        .onFalse(coral.stop());

    driverJoystick.R1().onTrue(coral.scoreCoral()).onFalse(coral.stop());

    driverJoystick
        .R2()
        .whileTrue(
            elevator
                .runElevatorTo(this::getTargetElevatorPosition)
                .andThen(Commands.waitUntil(elevator::isAtSetpoint)))
        .onFalse(elevator.runElevatorTo(ElevatorPosition.HOME));

    driverJoystick.L2().whileTrue(algae.intakeAlgae()).onFalse(algae.stopMotor());

    driverJoystick
        .L3()
        .toggleOnTrue(
            Commands.startEnd(
                () -> setThrottleProfile(ThrottleProfile.TURTLE),
                () -> setThrottleProfile(ThrottleProfile.TURBO)));

    driverJoystick.circle().whileTrue(algae.scoreAlgae()).onFalse(algae.stopMotor());
    driverJoystick
        .cross()
        .whileTrue(arm.runArmTo(ArmPosition.REEF).andThen(algae.intakeAlgae()))
        .onFalse(arm.runArmTo(ArmPosition.HOME).andThen(algae.stopMotor()));
    driverJoystick.square().onTrue(Commands.runOnce(drivetrain.orchestra::play));
    driverJoystick.triangle().onTrue(Commands.runOnce(drivetrain.orchestra::stop));

    driverJoystick
        .povUp()
        .whileTrue(BargeScoreCommand.raise(elevator, arm, algae))
        .whileFalse(BargeScoreCommand.lower(elevator, arm, algae));

    driverJoystick
        .povLeft()
        .whileTrue(
            elevator
                .runElevatorTo(this::getTargetElevatorPosition)
                .alongWith(new AutoReefAlignCommand(drivetrain, ReefTagPoses.ScoreSide.LEFT)))
        .whileFalse(
            Commands.either(
                Commands.idle(),
                elevator.runElevatorTo(ElevatorPosition.HOME),
                driverJoystick.R2()::getAsBoolean));

    driverJoystick
        .povRight()
        .whileTrue(
            new AutoReefAlignCommand(drivetrain, ReefTagPoses.ScoreSide.RIGHT)
                .alongWith(elevator.runElevatorTo(this::getTargetElevatorPosition)))
        .whileFalse(
            Commands.either(
                Commands.idle(),
                elevator.runElevatorTo(ElevatorPosition.HOME),
                driverJoystick.R2()::getAsBoolean));

    drivetrain.registerTelemetry(DriveConstants.TELEMETRY::telemeterize);
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));

    if (drivetrain.getPigeon2().getGravityVectorZ().getValue() >= 0.9) {
      driverJoystick.setRumble(RumbleType.kBothRumble, 0.5);
    }

    trigger.whileTrue(
        new InstantCommand(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 0.5)));
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
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
  }

  private void registerNamedCommands() {
    Object[] subsystems = new Object[] {coral, elevator, arm, algae};

    for (Object subsystem : subsystems) {
      registerNamedCommandsForSubsystem(subsystem);
    }

    for (ElevatorPosition position : ElevatorPosition.values()) {
      NamedCommands.registerCommand(
          "runElevatorTo" + position.toString(), elevator.runElevatorTo(position));

      NamedCommands.registerCommand(
          position.toString() + "ReefScoreCommand",
          ReefScoreCommand.get(position, elevator, coral));

      NamedCommands.registerCommand("intakeCoral", coral.intakeCoral());
    }

    for (ArmPosition position : ArmPosition.values()) {
      NamedCommands.registerCommand("runArmTo" + position.toString(), arm.runArmTo(position));
    }
  }

  private void registerNamedCommandsForSubsystem(Object subsystem) {
    for (int i = 0; i < subsystem.getClass().getDeclaredMethods().length; i++) {
      Method method = subsystem.getClass().getDeclaredMethods()[i];

      if (!method.getAnnotatedReturnType().getType().equals(Command.class)) {
        continue;
      }

      if (method.getParameterCount() > 0) {
        continue;
      }

      try {
        NamedCommands.registerCommand(method.getName(), (Command) method.invoke(subsystem));
      } catch (Exception exception) {
        throw new RuntimeException(exception);
      }
    }
  }

  public void disabledPeriodic() {
    Command selectedAutoCommand = autoChooser.getSelected();

    if (introspectedAutoCommand != selectedAutoCommand
        && selectedAutoCommand instanceof PathPlannerAuto) {
      autoStartPose =
          AllianceFlipUtil.apply(((PathPlannerAuto) selectedAutoCommand).getStartingPose());
      introspectedAutoCommand = selectedAutoCommand;
    }
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

  /**
   * Get the target elevator position.
   *
   * @return The position.
   */
  public ElevatorPosition getTargetElevatorPosition() {
    return targetElevatorPosition;
  }
}
