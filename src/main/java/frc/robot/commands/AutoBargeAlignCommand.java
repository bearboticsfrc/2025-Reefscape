package frc.robot.commands;

// import static frc.robot.constants.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceFlipUtil;

public class AutoBargeAlignCommand extends Command {
  private final double TRANSLATION_P = 6;
  private final double THETA_P = 10;

  private final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(6);
  private final LinearAcceleration TRANSLATION_MAX_ACCELERATION = MetersPerSecondPerSecond.of(4);

  private final AngularVelocity THETA_MAX_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
  private final AngularAcceleration THETA_MAX_ACCELERATION =
      RadiansPerSecondPerSecond.of(2 * Math.PI);

  private final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          TRANSLATION_MAX_VELOCITY.in(MetersPerSecond),
          TRANSLATION_MAX_ACCELERATION.in(MetersPerSecondPerSecond));

  private final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          THETA_MAX_VELOCITY.in(RadiansPerSecond),
          THETA_MAX_ACCELERATION.in(RadiansPerSecondPerSecond));

  private final SwerveRequest.FieldCentric DRIVE_TO_POSE = new FieldCentric();

  private static final Pose2d BLUE_SCORE_POSE = new Pose2d(8.092, 0, Rotation2d.k180deg);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(TRANSLATION_P, 0, 0, TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(THETA_P, 0, 0, THETA_CONSTRAINTS);

  private final CommandSwerveDrivetrain drivetrain;

  public AutoBargeAlignCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d targetPose =
        AllianceFlipUtil.apply(BLUE_SCORE_POSE)
            .plus(new Transform2d(0, currentPose.getX(), Rotation2d.kZero));

    xController.setGoal(targetPose.getX());
    thetaController.setGoal(targetPose.getRotation().getRadians());

    xController.reset(currentPose.getX());
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    double xVelocity = xController.calculate(currentPose.getX());
    double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians());

    drivetrain.setControl(
        DRIVE_TO_POSE
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withVelocityX(xVelocity)
            .withRotationalRate(thetaVelocity));
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && thetaController.atGoal();
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
