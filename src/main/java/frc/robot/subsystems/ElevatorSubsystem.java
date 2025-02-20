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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;
import java.io.IOException;

public class ElevatorSubsystem extends SubsystemBase {
  // Feedforward gains and motion parameters
  private final double A = 0;
  private final double S = 0.036;
  private final double G = 0.065;
  private final double V = 0;

  public final double MAX_ACCELERATION = 100; // was 35 on Tuesday;
  public final double MAX_VELOCITY = 70; // was 50 on Tuesday;

  // Elevator feedforward controller
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(A, G, S, V);

  // Trapezoidal motion profile constraints and instance
  private final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  // Spark motor controller instance
  private final SparkBase motor;
  private final RelativeEncoder encoder;

  /** Constructs a new ElevatorSubsystem by configuring the leader and follower motors. */
  public ElevatorSubsystem(CommandXboxController controller) {
    File baseDirectory = new File(Filesystem.getDeployDirectory(), "motors");
    File directory = new File(baseDirectory, "elevator");

    try {
      ConfiguredMotor configuredMotor =
          new MotorParser(directory).withMotor("leader.json").withPidf("pidf.json").configure();

      new MotorParser(directory).withMotor("follower.json").configure();

      motor = configuredMotor.getSpark();
      encoder = motor.getEncoder();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure elevator motor(s): ", exception);
    }

    ShuffleboardTab sensors = Shuffleboard.getTab("Sensors");

    sensors.addDouble("Output Current", motor::getOutputCurrent);
    sensors.addDouble("Applied Output", motor::getAppliedOutput);
    sensors.addDouble("Velocity", encoder::getVelocity);
    sensors.addDouble("Position", encoder::getPosition);
    sensors.addDouble("Setpoint State", () -> setpoint.position);
    sensors.addDouble("Goal State", () -> goal.position);
    sensors.addBoolean("Reverse Limit Switch", motor.getReverseLimitSwitch()::isPressed);
    sensors.addBoolean("Forward Limit Switch", motor.getForwardLimitSwitch()::isPressed);
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

  public Command stop() {
    return runOnce(() -> motor.stopMotor());
  }

  /** Enum representing preset elevator positions. */
  public enum ElevatorPosition {
    L4(39.6),
    L3(23.8),
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
