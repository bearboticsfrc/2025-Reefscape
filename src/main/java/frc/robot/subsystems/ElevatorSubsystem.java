package frc.robot.subsystems;

import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import java.io.File;
import java.io.IOException;

public class ElevatorSubsystem extends SubsystemBase {

  private ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KS, ElevatorConstants.KV);
  private ShuffleboardTab elevatorTab;
  private SparkBase leaderMotor;
  private RelativeEncoder leaderRelativeEncoder;
  private SparkClosedLoopController elevatorPIDController;
  private SparkBase followerMotor;
  private RelativeEncoder followerRelativeEncoder;
  private double elevatorState = ElevatorConstants.HOME;
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

      leaderMotor = elevatorLeaderMotorParser.configure().getSpark();

      MotorParser elevatorFollowerMotorParser =
          new MotorParser(directory)
              .withMotor("followerMotor.json")
              .withEncoder("followerEncoder.json")
              .withPidf("followerPidf.json");

      followerMotor = elevatorFollowerMotorParser.configure().getSpark(); 

    } catch (IOException e) {
      e.printStackTrace();
    }

    leaderRelativeEncoder = leaderMotor.getEncoder();
    followerRelativeEncoder = followerMotor.getEncoder();
    elevatorPIDController = leaderMotor.getClosedLoopController();
  }

  private void setupShuffleboardTab() {
    elevatorTab.add("position", elevatorState);
    elevatorTab.addDouble("leader encoder position", leaderRelativeEncoder::getPosition);
    elevatorTab.addDouble("follower encoder position", followerRelativeEncoder::getPosition);
  }

  public void setElevatorReference() {
    elevatorPIDController.setReference(elevatorState, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, elevatorFeedforward.calculate(ElevatorConstants.MAX_VELOCITY,ElevatorConstants.MAX_ACCELERATION));

  }

  public void setElevatorHome() {
    elevatorState = ElevatorConstants.HOME;
    setElevatorReference();
  
  }

  public void setElevatorL1() {
    elevatorState = ElevatorConstants.L1;
    setElevatorReference();
  
  }

  public void setElevatorL2() {
    elevatorState = ElevatorConstants.L2;
    setElevatorReference();

  }

  public void setElevatorL3() {
    elevatorState = ElevatorConstants.L3;
    setElevatorReference();

  }

  public void setElevatorL4() {
    elevatorState = ElevatorConstants.L4;
    setElevatorReference();
  
  }

}
