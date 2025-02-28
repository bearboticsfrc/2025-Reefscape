package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoCoralStationAlign extends Command {
    private final Map<Pose2d, Rotation2d> POSE_TO_ROTATION = Map.of();
    
    private final CommandSwerveDrivetrain drivetrain;
    
    private Rotation2d targetRotation;
    
    public AutoCoralStationAlign(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        // TODO :Keep going
        targetRotation = POSE_TO_ROTATION.get(drivetrain.getState().Pose.nearest(POSE_TO_ROTATION.keySet()));
    }
}
