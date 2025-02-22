package frc.robot.subsystems.manipulator;

import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {
  // Feedforward gains and motion parameters
  private final double A = 0;
  private final double S = 0.036;
  private final double G = 0.065;
  private final double V = 0;

  public final double MAX_ACCELERATION = 100; // was 35 on Tuesday;
  public final double MAX_VELOCITY = 70; // was 50 on Tuesday;

  // Spark motor controller instance
  @Logged(name = "Elevator Motor")
  private final SparkBase motor;

  @Logged(name = "Elevator Encoder")
  private final RelativeEncoder encoder;

  // Elevator feedforward controller
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(A, G, S, V);

  // Trapezoidal motion profile constraints and instance
  private final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);

  @Logged(name = "Arm Goal")
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  @Logged(name = "Arm Setpoint")
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  /** Constructs a new ElevatorSubsystem by configuring the leader and follower motors. */
  public ElevatorSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/elevator");

    try {
      motor =
          new MotorParser(directory)
              .withMotor("leader.json")
              .withPidf("pidf.json")
              .configureAsync();

      // Don't need to hold a reference, just configure...
      new MotorParser(directory).withMotor("follower.json").configureAsync();

      encoder = motor.getEncoder();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure elevator motor(s): ", exception);
    }
  }

  @Logged(name = "Elevator At Setpoint")
  public boolean isAtSetpoint() {
    return trapezoidProfile.timeLeftUntil(goal.position) == 0;
  }

  /**
   * Sets the target elevator position and updates the motor controller reference.
   *
   * @param position The desired elevator position.
   */
  private void set(ElevatorPosition position) {
    goal = new TrapezoidProfile.State(position.getPosition(), 0);
  }

  @Override
  public void periodic() {
    updateTrapezoidProfile();
  }

  /** Update the trapezoid motion profile setpoint. */
  private void updateTrapezoidProfile() {
    TrapezoidProfile.State nextSetpoint = trapezoidProfile.calculate(0.02, setpoint, goal);

    motor
        .getClosedLoopController()
        .setReference(
            setpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculateWithVelocities(setpoint.velocity, nextSetpoint.velocity));

    setpoint = nextSetpoint;
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

  /**
   * Creates a command to run the elevator to a specified position.
   *
   * @param position The target elevator position.
   * @return A command that moves the elevator.
   */
  public Command runElevatorTo(Supplier<ElevatorPosition> position) {
    return runOnce(() -> set(position.get()));
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor());
  }

  /** Enum representing preset elevator positions. */
  public enum ElevatorPosition {
    L4(39.6),
    L3(23.2),
    L2(12.5),
    L1(5),
    HOME(0);

    private final double position;

    ElevatorPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
