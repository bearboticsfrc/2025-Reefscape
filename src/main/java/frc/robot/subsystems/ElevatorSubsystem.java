package frc.robot.subsystems;

import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LinearControlEffLogCmd;
import frc.robot.constants.ElevatorConstants;
import java.io.File;
import java.io.IOException;

public class ElevatorSubsystem extends SubsystemBase {

  private ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(
          ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KS, ElevatorConstants.KV);
  private ShuffleboardTab elevatorTab;
  private SparkBase leaderMotor;
  private RelativeEncoder leaderRelativeEncoder;
  private SparkClosedLoopController elevatorPIDController;
  private SparkBase followerMotor;
  private RelativeEncoder followerRelativeEncoder;
  private double currentElevatorState = ElevatorConstants.HOME;
  private final boolean SHUFFLEBOARD_ENABLED = true;

  /*
   * constructor for the elevator subsystem
   */
  public ElevatorSubsystem() {
    elevatorTab = Shuffleboard.getTab("Elevator");
    configureMotors();
    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab();
    }
  }

  /*
   * configures the both  of motors for the elevator subsystem
   * along with their pid controllers and encoders
   */
  private void configureMotors() {
    File directory = new File(Filesystem.getDeployDirectory(), "Elevator");

    try {
      MotorParser elevatorLeaderMotorParser =
          new MotorParser(directory)
              .withMotor("leaderMotor.json")
              .withEncoder("leaderEncoder.json")
              .withPidf("leaderPidf.json", 0);

      leaderMotor = elevatorLeaderMotorParser.configure().getSpark();

      MotorParser elevatorFollowerMotorParser =
          new MotorParser(directory)
              .withMotor("followerMotor.json")
              .withEncoder("followerEncoder.json")
              .withPidf("followerPidf.json", 0);

      followerMotor = elevatorFollowerMotorParser.configure().getSpark();

    } catch (IOException e) {
      e.printStackTrace();
    }

    leaderRelativeEncoder = leaderMotor.getEncoder();
    followerRelativeEncoder = followerMotor.getEncoder();
    elevatorPIDController = leaderMotor.getClosedLoopController();
  }

  /*
   * configures the shuffleboard tab for the elevator subsystem and adds some values to it
   */
  private void setupShuffleboardTab() {
    elevatorTab.add("position", currentElevatorState);
    elevatorTab.addDouble("leader encoder position", leaderRelativeEncoder::getPosition);
    elevatorTab.addDouble("follower encoder position", followerRelativeEncoder::getPosition);
  }

  /*
   * sets the elevator reference to a target state
   * @param TargetElevatorState the target state for the elevator
   */
  public void setElevatorReference(double TargetElevatorState) {
    elevatorPIDController.setReference(
        TargetElevatorState,
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        elevatorFeedforward.calculate(
            elevatorFeedforward.maxAchievableVelocity(
                ElevatorConstants.MAX_VOLTAGE, ElevatorConstants.MAX_ACCELERATION)));
  }

  /*
   * sets the elevator to the home position
   */
  public void setElevatorHome() {
    setElevatorReference(ElevatorConstants.HOME);
    currentElevatorState = ElevatorConstants.HOME;
  }

  /*
   * sets the elevator to the reef level one position
   */
  public void setElevatorL1() {
    setElevatorReference(ElevatorConstants.L1);
    currentElevatorState = ElevatorConstants.L1;
  }

  /*
   * sets the elevator to the reef level two position
   */
  public void setElevatorL2() {
    setElevatorReference(ElevatorConstants.L2);
    currentElevatorState = ElevatorConstants.L2;
  }

  /*
   * sets the elevator to the reef level three position
   */
  public void setElevatorL3() {
    setElevatorReference(ElevatorConstants.L3);
    currentElevatorState = ElevatorConstants.L3;
  }

  /*
   * sets the elevator to the reef level four position
   */
  public void setElevatorL4() {
    setElevatorReference(ElevatorConstants.L4);
    currentElevatorState = ElevatorConstants.L4;
  }

  /**
   * @param elevatorCommand a command run on the elevator logs the data from the command. Work done
   *     by motor, avg power pulled, Theoretical Work requirement
   */
  public Command logElevatorCommandData(Command elevatorCommand, String commandName) {
    LinearControlEffLogCmd loggingCmd =
        new LinearControlEffLogCmd(new SparkBase[] {leaderMotor, followerMotor});

    return new ParallelRaceGroup(elevatorCommand, loggingCmd);
  }
}
