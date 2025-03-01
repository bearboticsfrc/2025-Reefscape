package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NTSubsystem extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;

  @Logged public Field2d field = new Field2d();

  public NTSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    field.setRobotPose(drivetrain.getState().Pose);
  }
  
  @Logged
  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }
}
