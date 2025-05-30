// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ProcessedJoystick.JoystickAxis;
import frc.robot.ProcessedJoystick.ThrottleProfile;
import frc.robot.commands.AutoAlgaePickupCommand;
import frc.robot.commands.AutoBargeAlignCommand;
import frc.robot.commands.AutoCoralStationAlignCommand;
import frc.robot.commands.AutoReefAlignCommand;
import frc.robot.commands.BargeScoreCommand;
import frc.robot.commands.ProccessorScoreCommand;
import frc.robot.commands.ReefScoreCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.reef.ReefTagPoses;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.AlgaeSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.manipulator.CoralSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utils.AllianceFlipUtil;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

public class RobotContainer {
  private final CommandXboxController driverJoystick =
      new CommandXboxController(DriveConstants.DRIVER_JOYSTICK_PORT);

  private final CommandGenericHID operatorGamepad =
      new CommandGenericHID(DriveConstants.OPERATOR_GAMEPAD_PORT);

  private final ProcessedJoystick processedJoystick =
      new ProcessedJoystick(driverJoystick, this::getThrottleProfile, DriveConstants.MAX_VELOCITY);

  @Logged(importance = Importance.CRITICAL)
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged(importance = Importance.CRITICAL)
  private final CoralSubsystem coral = new CoralSubsystem();

  @Logged(importance = Importance.CRITICAL)
  private final AlgaeSubsystem algae = new AlgaeSubsystem();

  @Logged(importance = Importance.CRITICAL)
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  @Logged(importance = Importance.CRITICAL)
  private final ArmSubsystem arm = new ArmSubsystem();

  private ProcessedJoystick.ThrottleProfile throttleProfile =
      ProcessedJoystick.ThrottleProfile.TURBO;

  private SendableChooser<Command> autoChooser;

  @Logged(name = "Auto Start Pose", importance = Importance.CRITICAL)
  private Pose2d autoStartPose;

  private Command introspectedAutoCommand;

  @Logged(name = "Target Elevator Position", importance = Importance.CRITICAL)
  private ElevatorPosition targetElevatorPosition = ElevatorPosition.HOME;

  private final CANdleSubsystem CANdle =
      new CANdleSubsystem(elevator, () -> targetElevatorPosition);

  public RobotContainer() {
    configureDriverXboxBindings();
    configureOperatorBindings();
    configureTriggers();

    configureAutoBuilder();
  }

  /** Configure the driver control bindings. */
  private void configureDriverPS4Bindings(CommandPS4Controller driverJoystick) {
    driverJoystick.L1().whileTrue(coral.intakeCoral()).onFalse(coral.stop());

    driverJoystick.R1().onTrue(coral.teleopScoreCore()).onFalse(coral.stop());

    driverJoystick
        .povDown()
        .whileTrue(
            elevator
                .runElevatorTo(this::getTargetElevatorPosition)
                .andThen(Commands.waitUntil(elevator::isAtSetpoint))
                .unless(coral::intakeHasCoral))
        .onFalse(elevator.runElevatorTo(ElevatorPosition.HOME));

    driverJoystick
        .L2()
        .whileTrue(new AutoCoralStationAlignCommand(drivetrain).alongWith(coral.intakeCoral()))
        .onFalse(coral.stop());

    driverJoystick
        .L3()
        .toggleOnTrue(
            Commands.startEnd(
                () -> setThrottleProfile(ThrottleProfile.TURTLE),
                () -> setThrottleProfile(ThrottleProfile.TURBO)));

    driverJoystick
        .circle()
        .debounce(0.15)
        .onTrue(new ProccessorScoreCommand(algae, arm).unless(driverJoystick.R2()::getAsBoolean));

    driverJoystick
        .cross()
        .debounce(0.15)
        .whileTrue(
            new AutoAlgaePickupCommand(drivetrain, algae, arm, elevator)
                .unless(driverJoystick.R2()::getAsBoolean));

    driverJoystick
        .povUp()
        .whileTrue(bargeAlignCommand().andThen(BargeScoreCommand.raise(elevator, arm, algae)))
        .whileFalse(BargeScoreCommand.lower(elevator, arm, algae));

    driverJoystick
        .povLeft()
        .whileTrue(
            elevator
                .runElevatorTo(this::getTargetElevatorPosition)
                .alongWith(new AutoReefAlignCommand(drivetrain, ReefTagPoses.ScoreSide.LEFT)))
        .whileFalse(
            elevator
                .runElevatorTo(ElevatorPosition.HOME)
                .unless(driverJoystick.R2()::getAsBoolean));

    driverJoystick
        .povRight()
        .whileTrue(
            new AutoReefAlignCommand(drivetrain, ReefTagPoses.ScoreSide.RIGHT)
                .alongWith(elevator.runElevatorTo(this::getTargetElevatorPosition)))
        .whileFalse(
            elevator
                .runElevatorTo(ElevatorPosition.HOME)
                .unless(driverJoystick.R2()::getAsBoolean));

    driverJoystick
        .R2()
        .and(driverJoystick.triangle())
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L4)))
        .onTrue(Commands.runOnce(CANdle::reset));

    driverJoystick
        .R2()
        .and(driverJoystick.circle())
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L3)))
        .onTrue(Commands.runOnce(CANdle::reset));

    driverJoystick
        .R2()
        .and(driverJoystick.cross())
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L2)))
        .onTrue(Commands.runOnce(CANdle::reset));

    drivetrain.registerTelemetry(DriveConstants.TELEMETRY::telemeterize);
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));
  }

  private void configureDriverXboxBindings() {
    driverJoystick.leftBumper().whileTrue(coral.intakeCoral()).onFalse(coral.stop());

    driverJoystick.rightBumper().onTrue(coral.teleopScoreCore()).onFalse(coral.stop());

    driverJoystick
        .povDown()
        .whileTrue(
            elevator
                .runElevatorTo(this::getTargetElevatorPosition)
                .andThen(Commands.waitUntil(elevator::isAtSetpoint))
                .unless(coral::intakeHasCoral))
        .onFalse(elevator.runElevatorTo(ElevatorPosition.HOME));

    driverJoystick
        .leftTrigger()
        .whileTrue(new AutoCoralStationAlignCommand(drivetrain).alongWith(coral.intakeCoral()))
        .onFalse(coral.stop());

    driverJoystick
        .leftStick()
        .toggleOnTrue(
            Commands.startEnd(
                () -> setThrottleProfile(ThrottleProfile.TURTLE),
                () -> setThrottleProfile(ThrottleProfile.TURBO)));

    driverJoystick
        .b()
        .debounce(0.15)
        .onTrue(
            new ProccessorScoreCommand(algae, arm)
                .unless(driverJoystick.rightTrigger()::getAsBoolean));

    driverJoystick
        .a()
        .debounce(0.15)
        .whileTrue(
            new AutoAlgaePickupCommand(drivetrain, algae, arm, elevator)
                .unless(driverJoystick.rightTrigger()::getAsBoolean));

    driverJoystick
        .povUp()
        .whileTrue(bargeAlignCommand().andThen(BargeScoreCommand.raise(elevator, arm, algae)))
        .whileFalse(BargeScoreCommand.lower(elevator, arm, algae));

    driverJoystick
        .povLeft()
        .whileTrue(
            elevator
                .runElevatorTo(this::getTargetElevatorPosition)
                .alongWith(new AutoReefAlignCommand(drivetrain, ReefTagPoses.ScoreSide.LEFT)))
        .whileFalse(
            elevator
                .runElevatorTo(ElevatorPosition.HOME)
                .unless(driverJoystick.rightTrigger()::getAsBoolean));

    driverJoystick
        .povRight()
        .whileTrue(
            new AutoReefAlignCommand(drivetrain, ReefTagPoses.ScoreSide.RIGHT)
                .alongWith(elevator.runElevatorTo(this::getTargetElevatorPosition)))
        .whileFalse(
            elevator
                .runElevatorTo(ElevatorPosition.HOME)
                .unless(driverJoystick.rightTrigger()::getAsBoolean));

    driverJoystick
        .rightTrigger()
        .and(driverJoystick.y())
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L4)))
        .onTrue(Commands.runOnce(CANdle::reset));

    driverJoystick
        .rightTrigger()
        .and(driverJoystick.b())
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L3)))
        .onTrue(Commands.runOnce(CANdle::reset));

    driverJoystick
        .rightTrigger()
        .and(driverJoystick.a())
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L2)))
        .onTrue(Commands.runOnce(CANdle::reset));

    drivetrain.registerTelemetry(DriveConstants.TELEMETRY::telemeterize);
    drivetrain.setDefaultCommand(drivetrain.applyRequest(this::getDefaultDriveRequest));
  }

  /** Configure the operator control bindings. */
  private void configureOperatorBindings() {
    operatorGamepad
        .button(1)
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L1)))
        .onTrue(Commands.runOnce(CANdle::reset));

    operatorGamepad
        .button(2)
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L2)))
        .onTrue(Commands.runOnce(CANdle::reset));

    operatorGamepad
        .button(3)
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L3)))
        .onTrue(Commands.runOnce(CANdle::reset));

    operatorGamepad
        .button(4)
        .onTrue(Commands.runOnce(() -> setTargetElevatorPosition(ElevatorPosition.L4)))
        .onTrue(Commands.runOnce(CANdle::reset));
  }

  private void configureTriggers() {
    new Trigger(coral::hasCoral).and(coral::isIntakeActive).onTrue(CANdle.setCoralStrobeCommand());
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
        .withRotationalRate(processedJoystick.get(JoystickAxis.Rx) * 1.33);
  }

  /** Configures the PathPlanner autobuilder for publishing on NT. */
  private void configureAutoBuilder() {
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /** Register all named commands for each subsystem. */
  private void registerNamedCommands() {
    final SubsystemBase[] subsystems = new SubsystemBase[] {coral, elevator};

    for (SubsystemBase subsystem : subsystems) {
      registerNamedCommandsForSubsystem(subsystem);
    }

    for (ElevatorPosition position : ElevatorPosition.values()) {
      final String elevatorPosition = position.toString();

      NamedCommands.registerCommand(
          "runElevatorTo" + elevatorPosition,
          Commands.waitUntil(coral::outakeHasCoral).andThen(elevator.runElevatorTo(position)));

      NamedCommands.registerCommand(
          "fullyRunElevatorTo" + elevatorPosition,
          elevator.runElevatorTo(position).andThen(Commands.waitUntil(elevator::isAtSetpoint)));

      NamedCommands.registerCommand(
          elevatorPosition + "ReefScoreCommand", new ReefScoreCommand(position, elevator, coral));
    }

    for (ArmPosition position : ArmPosition.values()) {
      final String armPosition = position.toString();

      NamedCommands.registerCommand("runArmTo" + armPosition, arm.runArmTo(position));
    }

    NamedCommands.registerCommand("runIntake", coral.intakeCoral());

    NamedCommands.registerCommand(
        "waitUntilIntakeHasCoral", Commands.waitUntil(coral::intakeHasCoral));

    NamedCommands.registerCommand(
        "intakeAlgae", arm.runArmTo(ArmPosition.REEF).andThen(algae.intakeAlgae()));

    NamedCommands.registerCommand(
        "BargeScoreCommand",
        BargeScoreCommand.raise(elevator, arm, algae)
            .andThen(elevator.runElevatorTo(ElevatorPosition.L2)));
  }

  /**
   * Register all named commands for the subsystems.
   *
   * <p>This iterates over all declared methods in the subsystem and registers commands which adhear
   * to the conditions:
   *
   * <ul>
   *   <li>The annotated return type is {@link Command}.
   *   <li>The parameter count is 0.
   * </ul>
   *
   * @param subsystem The subsystem to check,
   */
  private void registerNamedCommandsForSubsystem(SubsystemBase subsystem) {
    for (int i = 0; i < subsystem.getClass().getDeclaredMethods().length; i++) {
      Method method = subsystem.getClass().getDeclaredMethods()[i];

      if (!method.getAnnotatedReturnType().getType().equals(Command.class)) {
        continue;
      }

      if (method.getParameterCount() > 0) {
        continue;
      }

      if (Modifier.isPrivate(method.getModifiers())) {
        continue;
      }

      try {
        NamedCommands.registerCommand(method.getName(), (Command) method.invoke(subsystem));
      } catch (Exception exception) {
        throw new RuntimeException(exception);
      }
    }
  }

  /**
   * Teleop Initialization.
   *
   * <p>Tares the elevator and lowers to {@code ElevatorPosition.HOME} if the arm is at {@code
   * ArmPosition.HOME}, otherwise keep it at the same goal.
   */
  public void teleopInit() {
    final boolean isArmPositionNearHome =
        MathUtil.isNear(ArmPosition.HOME.getPosition(), arm.getPosition(), 5);

    if (isArmPositionNearHome && !algae.hasAlgae()) {
      arm.set(ArmPosition.HOME);
      elevator.set(ElevatorPosition.HOME);
    }

    coral.stop().schedule();
    algae.stopMotor().schedule();

    arm.tareClosedLoopController();
    elevator.tareClosedLoopController();
  }

  /**
   * Robot Initialization.
   *
   * <p>Resets the LEDs.
   */
  public void robotInit() {
    CANdle.reset();
  }

  /** Disabled periodic which updates the autonomous starting pose. */
  public void disabledPeriodic() {
    Command selectedAutoCommand = autoChooser.getSelected();

    if (introspectedAutoCommand != selectedAutoCommand
        && selectedAutoCommand instanceof PathPlannerAuto) {
      autoStartPose =
          AllianceFlipUtil.apply(((PathPlannerAuto) selectedAutoCommand).getStartingPose());
      introspectedAutoCommand = selectedAutoCommand;
    }
  }

  private Command bargeAlignCommand() {
    final Command elevatorRaiseCommand =
        elevator
            .runElevatorTo(ElevatorPosition.L2)
            .onlyIf(() -> elevator.getPosition() == ElevatorPosition.HOME.getPosition());

    return new AutoBargeAlignCommand(drivetrain)
        .alongWith(
            Commands.waitUntil(() -> AutoBargeAlignCommand.isNearBarge(drivetrain.getState().Pose))
                .andThen(elevatorRaiseCommand));
  }

  /**
   * Sets the target elevator position.
   *
   * @param position The target position.
   */
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
