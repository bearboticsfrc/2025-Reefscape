package frc.robot.subsystems;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

//TODO: ask kellen about how to configure absolute encoders
public class ElevatorSubsystem extends SubsystemBase {

  private ShuffleboardTab elevatorTab;
  private SparkBase leaderMotor;
  private RelativeEncoder leaderRelativeEncoder;
  private AbsoluteEncoder leaderAbsoluteEncoder;
  private SparkBase followerMotor;
  private RelativeEncoder followerRelativeEncoder;
  private AbsoluteEncoder followerAbsoluteEncoder;
  private ElevatorState elevatorState = ElevatorState.Home;
  private final boolean SHUFFLEBOARD_ENABLED = true;

  public ElevatorSubsystem() {
    elevatorTab = Shuffleboard.getTab("Elevator");
    configureMotors();
    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab();
    }
  }

  private void configureMotors() {
    File directory = new File(Filesystem.getDeployDirectory(), "Elevator");

       try {
    MotorParser elevatorLeaderMotorParser =
        new MotorParser(directory)
            .withMotor("leaderMotor.json")
            .withEncoder("leaderEncoder.json")
            .withPidf("leaderPidf.json");

    ConfiguredMotor configuredLeaderMotor = elevatorLeaderMotorParser.configure();

    // TODO: Configure SparkMax objects and assign it to LeaderMotor instead of SparkBase
    leaderMotor = configuredLeaderMotor.getSpark();

    MotorParser elevatorFollowerMotorParser =
        new MotorParser(directory)
            .withMotor("followerMotor.json")
            .withEncoder("followerEncoder.json")
            .withPidf("followerPidf.json");

    ConfiguredMotor configuredFollowerMotor = elevatorFollowerMotorParser.configure();

    // TODO: Configure SparkMax objects and assign it to FollowerMotor instead of SparkBase
    followerMotor = configuredFollowerMotor.getSpark();

     } catch (IOException e) {
        e.printStackTrace();
      }

    leaderAbsoluteEncoder = leaderMotor.getAbsoluteEncoder();
    leaderRelativeEncoder = leaderMotor.getEncoder();
    followerAbsoluteEncoder = followerMotor.getAbsoluteEncoder();
    followerRelativeEncoder = followerMotor.getEncoder();
  }

  private void setupShuffleboardTab() {
    elevatorTab.add("four", 4.0);
  }

  enum ElevatorState {
    Home,
    L1,
    L2,
    L3,
    L4;
  }
}
