package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.reef.ReefTagPoses;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command to drive the robot to the nearest scoring pose on a specified side of the reef. It
 * configures a DriveToPoseCommand with specific PID gains and tolerances.
 */
public class AutoReefAlignCommand extends DriveToPoseCommand {

  private static final double TRANSLATION_P = 4;
  private static final double TRANSLATION_I = 0;
  private static final double TRANSLATION_D = 0;

  private static final Distance TRANSLATION_TOLERANCE = Centimeters.of(3);
  private static final Angle HEADING_TOLERANCE = Degrees.of(2);

  private final CommandSwerveDrivetrain drivetrain;
  private final ReefTagPoses.ScoreSide side;

  /**
   * Creates a new AutoReefAlignCommand.
   *
   * @param drivetrain The drivetrain subsystem.
   * @param side The side of the reef to align to (LEFT or RIGHT).
   */
  public AutoReefAlignCommand(CommandSwerveDrivetrain drivetrain, ReefTagPoses.ScoreSide side) {
    super(drivetrain, () -> drivetrain.getState().Pose); // Initial pose supplier, will be updated
    this.drivetrain = drivetrain;
    this.side = side;

    super.withTranslationPID(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D)
        .withTranslationTolerance(TRANSLATION_TOLERANCE)
        .withHeadingTolerance(HEADING_TOLERANCE)
        .withPoseSupplier(this::getTargetPose);

    setName(getClass().getName() + "(" + side.toString() + ")");
  }

  public Pose2d getTargetPose() {
    return ReefTagPoses.getNearestScoringPose(drivetrain.getState().Pose, side);
  }
}
