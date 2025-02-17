package frc.robot.subsystems;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.deserializer.MotorParser;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;

import javax.xml.crypto.Data;

import edu.wpi.first.wpilibj.DataLogManager;

public class ElevatorSubsystem extends SubsystemBase {
  // Time step for trapezoidal profile calculations (in seconds)
  private static final double DT = 0.02;

  // Feedforward gains and motion parameters
  private static final double A = 0.41;
  private static final double S = 0.0;
  private static final double G = 2.26;
  private static final double V = 3.08;

  public final double MAX_ACCELERATION = 0.3;
  public final double MAX_VOLTAGE = 0.0;

  // Trapezoidal motion profile constraints and instance
  private final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(V, A);
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);

  // Elevator feedforward controller
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(A, G, S, V);

  // Spark motor controller instance
  private final SparkBase motor;

  private final SparkBase follower;

  // @Logged(importance = Importance.DEBUG)
  private TrapezoidProfile.State targetState = new TrapezoidProfile.State(0, 0);

  // @Logged(importance = Importance.DEBUG)
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State(0, 0);

  /** Constructs a new ElevatorSubsystem by configuring the leader and follower motors. */
  public ElevatorSubsystem() {
    File baseDirectory = new File(Filesystem.getDeployDirectory(), "motors/elevator");
    File leaderDirectory = new File(baseDirectory, "leader");
    File followerDirectory = new File(baseDirectory, "follower");

    try {
      ConfiguredMotor configuredLeaderMotor =
          new MotorParser(leaderDirectory)
              .withMotor("motor.json")
              .withEncoder("encoder.json")
              .withPidf("pidf.json")
              .configure();

      follower = new MotorParser(followerDirectory).withMotor("motor.json").configure().getSpark();

      // follower configuration
      // new MotorParser(followerDirectory).withMotor("motor.json").configure();

      motor = configuredLeaderMotor.getSpark();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure elevator motor(s)!", exception);
    }
  }

  /**
   * Sets the target elevator position and updates the motor controller reference.
   *
   * @param position The desired elevator position.
   */
  private void set(ElevatorPosition position) {
    targetState = new TrapezoidProfile.State(position.getPosition(), 0);
    currentState = calculateState(targetState);

    motor
        .getClosedLoopController()
        .setReference(
            currentState.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            calculateFeedForward());
  }

  /**
   * Calculates the trapezoidal profile state based on the current encoder position and target
   * state.
   *
   * @param targetState The target state for the motion profile.
   * @return The updated trapezoidal profile state.
   */
  private TrapezoidProfile.State calculateState(TrapezoidProfile.State targetState) {
    double currentPosition = motor.getAbsoluteEncoder().getPosition();
    return trapezoidProfile.calculate(
        DT, new TrapezoidProfile.State(currentPosition, currentState.velocity), targetState);
  }

  /**
   * Calculates the feedforward value using the current encoder position and velocity.
   *
   * @return The computed feedforward value.
   */
  private double calculateFeedForward() {
    double currentPosition = motor.getAbsoluteEncoder().getPosition();
    return feedforward.calculate(currentPosition, currentState.velocity);
  }

  @Override
  public void periodic() {
    currentState = trapezoidProfile.calculate(DT, currentState, targetState);
    double feedForwardValue = feedforward.calculate(currentState.position, currentState.velocity);

    motor
        .getClosedLoopController()
        .setReference(
            currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardValue);
    SmartDashboard.putNumber("Elevator encoder", motor.getEncoder().getPosition());
  }

  /**
   * Creates a command to run the elevator to a specified position.
   *
   * @param position The target elevator position.
   * @return A command that moves the elevator.
   */
  private Command runElevatorTo(ElevatorPosition position) {
    return runOnce(() -> set(position));
  }

  /** Enum representing preset elevator positions. */
  public enum ElevatorPosition {
    L4(0),
    L3(0),
    L2(0),
    L1(0),
    HOME(0);

    private final double position;

    ElevatorPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public Command zeroRelativeEncoder() {
    return Commands.runOnce(() -> motor.getEncoder().setPosition(0));
  }

  public void setSpeed(double power) {
    DataLogManager.log("setSpeed(): " + power);
   //motor.set(power);
  }
}
