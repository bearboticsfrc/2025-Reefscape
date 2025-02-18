package frc.robot.subsystems;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class ElevatorSubsystem extends SubsystemBase {
  // Feedforward gains and motion parameters
  private final double A = 0;
  private final double S = 0;
  private final double G = 0.042;
  private final double V = 0;

  public final double MAX_ACCELERATION = 0.1;
  public final double MAX_VELOCITY = 1;

  // Trapezoidal motion profile constraints and instance
  private final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);

  // Elevator feedforward controller
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(A, G, S, V);

  private TrapezoidProfile.State targetState = new TrapezoidProfile.State();
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State();

  // Spark motor controller instance
  private final SparkBase motor;
  private final RelativeEncoder encoder;

  /** Constructs a new ElevatorSubsystem by configuring the leader and follower motors. */
  public ElevatorSubsystem() {
    File baseDirectory = new File(Filesystem.getDeployDirectory(), "motors");
    File directory = new File(baseDirectory, "elevator");

    try {
      ConfiguredMotor configuredMotor =
          new MotorParser(directory)
              .withMotor("leader.json")
              .withEncoder("encoder.json")
              .withPidf("pidf.json")
              .configure();

      new MotorParser(directory).withMotor("follower.json").configure();

      motor = configuredMotor.getSpark();
      encoder = configuredMotor.getRelativeEncoder().get();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure elevator motor(s): ", exception);
    }

    Shuffleboard.getTab("Sensors").addDouble("Applied Output", motor::getAppliedOutput);
    Shuffleboard.getTab("Sensors").addDouble("Position", encoder::getPosition);
    Shuffleboard.getTab("Sensors").addDouble("Current State", () -> currentState.position);
    Shuffleboard.getTab("Sensors").addDouble("Target State", () -> targetState.position);
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
            calculateFeedForward(targetState));
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
        0.02, new TrapezoidProfile.State(currentPosition, currentState.velocity), targetState);
  }

  /**
   * Calculates the feedforward value using the current encoder position and velocity.
   *
   * @return The computed feedforward value.
   */
  private double calculateFeedForward(TrapezoidProfile.State nextState) {
    return feedforward.calculateWithVelocities(currentState.velocity, nextState.velocity);
  }

  @Override
  public void periodic() {
    TrapezoidProfile.State nextState = trapezoidProfile.calculate(0.02, currentState, targetState);
    double feedForwardValue = calculateFeedForward(nextState);

    motor
        .getClosedLoopController()
        .setReference(
            currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardValue);

    currentState = nextState;
  }

  /**
   * Creates a command to run the elevator to a specified position.
   *
   * @param position The target elevator position.
   * @return A command that moves the elevator.
   */
  public Command runElevatorTo(ElevatorPosition position) {
    return runOnce(() -> set(position));
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor());
  }

  /** Enum representing preset elevator positions. */
  public enum ElevatorPosition {
    L4(38),
    L3(0),
    L2(15),
    L1(10),
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
    motor.set(power);
  }
}
