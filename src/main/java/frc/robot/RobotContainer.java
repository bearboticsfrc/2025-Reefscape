// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.NTSubsystem;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.CoralSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;
import java.lang.reflect.Method;

public class RobotContainer {
  private final CommandPS4Controller driverJoystick =
      new CommandPS4Controller(DriveConstants.DRIVER_JOYSTICK_PORT);

  private final CommandXboxController operatorJoystick =
      new CommandXboxController(DriveConstants.OPERATOR_JOYSTICK_PORT);

  private final CommandGenericHID operatorGamepad =
      new CommandGenericHID(DriveConstants.OPERATOR_GAMEPAD_PORT);

  private final ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile, DriveConstants.MAX_VELOCITY);

  private ProcessedJoystick.ThrottleProfile throttleProfile =
      ProcessedJoystick.ThrottleProfile.TURBO;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged private final CoralSubsystem coral = new CoralSubsystem();
  @Logged private final AlgaeSubsystem algae = new AlgaeSubsystem();
  @Logged private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  @Logged private final ArmSubsystem arm = new ArmSubsystem();

  @Logged private final NTSubsystem NTSubsystem = new NTSubsystem(drivetrain);

  private SendableChooser<Command> autoChooser;

  @Logged private ElevatorPosition targetElevatorPosition = ElevatorPosition.HOME;

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
            coral
                .intakeCoral()
                .alongWith(
                    new AutoCoralStationAlign(
                        drivetrain,
                        () -> processedJoystick.get(JoystickAxis.Ly),
                        () -> processedJoystick.get(JoystickAxis.Lx))))
        .onFalse(coral.stop());

    driverJoystick.R1().onTrue(coral.scoreCoral()).onFalse(coral.stop());

    driverJoystick
        .R2()
        .whileTrue(
            elevator
                .runElevatorTo(() -> targetElevatorPosition)
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
            new AutoReefAlignCommand(drivetrain, ReefTagPoses.ScoreSide.LEFT)
                .alongWith(elevator.runElevatorTo(() -> targetElevatorPosition)))
        .whileFalse(elevator.runElevatorTo(ElevatorPosition.HOME));

    driverJoystick
        .povRight()
        .whileTrue(
            new AutoReefAlignCommand(drivetrain, ReefTagPoses.ScoreSide.RIGHT)
                .alongWith(elevator.runElevatorTo(() -> targetElevatorPosition)))
        .whileFalse(elevator.runElevatorTo(ElevatorPosition.HOME));

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
    SmartDashboard.putData("Auto Path", autoChooser);
  }

  private void registerNamedCommands() {
    Object[] subsystems = new Object[] {coral, elevator, arm, algae};

    for (Object subsystem : subsystems) {
      for (Method method : subsystems.getClass().getDeclaredMethods()) {
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

    for (ElevatorPosition position : ElevatorPosition.values()) {
      NamedCommands.registerCommand(
          "runElevatorTo" + position.toString(), elevator.runElevatorTo(position));
      NamedCommands.registerCommand(
          position.toString() + "ReefScoreCommand",
          ReefScoreCommand.get(position, elevator, coral));
    }

    for (ArmPosition position : ArmPosition.values()) {
      NamedCommands.registerCommand("runArmTo" + position.toString(), arm.runArmTo(position));
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
}
