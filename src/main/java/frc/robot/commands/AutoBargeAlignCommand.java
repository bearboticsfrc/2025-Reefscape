package frc.robot.commands;

// import static frc.robot.constants.*;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoBargeAlignCommand extends Command {
  public final double blueBargeX = 8;
  public final double redBargeX = 9.4;
  public final double blueBargeTheta = 180;
  public final double redBargeTheta = 0;

  private final double TRANSLATION_K = 6;
  private final double THETA_K = 10;

  private final double TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(6).in(MetersPerSecond);
  private final double TRANSLATION_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(4).in(MetersPerSecondPerSecond);

  private final double THETA_MAX_VELOCITY = RadiansPerSecond.of(2 * Math.PI).in(RadiansPerSecond);
  private final double THETA_MAX_ACCELERATION =
      RadiansPerSecondPerSecond.of(2 * Math.PI).in(RadiansPerSecondPerSecond);

  private final double POSITION_TOLERANCE = Inches.of(1).in(Meters);
  private final double ROTATIONS_TOLERANCE = Radians.of(0.017).in(Radians);

  private final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION);
  private final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(THETA_MAX_VELOCITY, THETA_MAX_ACCELERATION);

  private final SwerveRequest.FieldCentric DRIVE_TO_POSE = new FieldCentric();

  private final ProfiledPIDController xController =
      new ProfiledPIDController(TRANSLATION_K, 0, 0, TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(THETA_K, 0, 0, THETA_CONSTRAINTS);

  private final CommandSwerveDrivetrain drivetrain;

  private Rotation2d targetRotation;

  private final FieldCentricFacingAngle swerveRequest = new FieldCentricFacingAngle();

  public AutoBargeAlignCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    ;
  }

  @Override
  public void initialize() {

    swerveRequest.TargetDirection = targetRotation;
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    final double xVelocity = xController.calculate(currentPose.getX());
    final double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians());

    drivetrain.setControl(
        DRIVE_TO_POSE
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withVelocityX(xVelocity)
            .withRotationalRate(thetaVelocity));
  }
}
