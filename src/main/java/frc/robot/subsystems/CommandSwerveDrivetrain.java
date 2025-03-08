package frc.robot.subsystems;

import bearlib.fms.AllianceColor;
import bearlib.fms.AllianceReadyListener;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.vision.Vision;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain
    implements Subsystem, AllianceReadyListener {
  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.k180deg;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  @Logged
  private final Vision vision =
      new Vision(
          Arrays.asList(
              VisionConstants.FRONT_LEFT_CAMERA,
              VisionConstants.FRONT_RIGHT_CAMERA,
              VisionConstants.REAR_CAMERA));

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    AllianceColor.addListener(this);
    configureAutoBuilder();
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> setControl(requestSupplier.get()));
  }

  @Override
  public void periodic() {
    List<EstimatedRobotPose> estimatedPoses = vision.getEstimatedGlobalPoses();

    for (EstimatedRobotPose estimatedPose : estimatedPoses) {
      addVisionMeasurement(estimatedPose);
    }
  }

  /**
   * Adds vision-based pose estimation measurements to the drivetrain.
   *
   * @param estimatedRobotPose The estimated robot pose from vision processing.
   */
  public void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose) {
    Pose2d estPose = estimatedRobotPose.estimatedPose.toPose2d();

    addVisionMeasurement(estPose, Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds));
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose,
          this::resetPose,
          () -> getState().Speeds,
          this::setChassisSpeeds,
          new PPHolonomicDriveController(new PIDConstants(10, 0, 0), new PIDConstants(7, 0, 0)),
          config,
          () -> AllianceColor.getAlliance() == Alliance.Red,
          this);
    } catch (Exception exception) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", exception.getStackTrace());
    }
  }

  private void setChassisSpeeds(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    setControl(
        pathApplyRobotSpeeds
            .withSpeeds(speeds)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
  }

  /**
   * Updates the field-centric driving perspective of the operator based on the robot's current
   * alliance. The perspective is set to face forward relative to the alliance's side of the field.
   *
   * @param alliance The current alliance (Red or Blue).
   */
  @Override
  public void updateAlliance(Alliance alliance) {
    setOperatorPerspectiveForward(
        alliance == Alliance.Red
            ? redAlliancePerspectiveRotation
            : blueAlliancePerspectiveRotation);
  }
}
