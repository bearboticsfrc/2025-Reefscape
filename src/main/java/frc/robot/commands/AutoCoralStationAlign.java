package frc.robot.commands;

import static frc.robot.constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoCoralStationAlign extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    
    private final Map<Pose2d, Rotation2d> poseToRotation;

    private Rotation2d targetRotation;
    
    public AutoCoralStationAlign(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.poseToRotation = getPoseToRotation();
    }

    private Map<Pose2d, Rotation2d> getPoseToRotation() {
        Map<Pose2d, Rotation2d> poseToRotation = new HashMap<>();

        for (AprilTag tag : APRIL_TAG_FIELD_LAYOUT.getTags()) {
            if (tag.ID != 1 || tag.ID != 2 || tag.ID != 12 || tag.ID != 13) {
                continue;
            }

            Pose2d tagPose = tag.pose.toPose2d();
            poseToRotation.put(tagPose, Rotation2d.fromDegrees(180 - tagPose.getRotation().getDegrees()));
        }

        return poseToRotation;
    }

    @Override
    public void initialize() {
        // TODO :Keep going
        targetRotation = poseToRotation.get(drivetrain.getState().Pose.nearest(new ArrayList<>(poseToRotation.keySet())));
    }
}
