package frc.robot.subsystems;

import bearlib.motor.ConfiguredMotor;
import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;
import java.io.IOException;

public class ArmSubsystem extends SubsystemBase {
  // Feedforward gains and motion parameters
  private final double A = 0;
  private final double S = 0.010;
  private final double G = 0.0095; // / or maybe .0145???
  private final double V = 0;

  public final double MAX_ACCELERATION = .5;
  public final double MAX_VELOCITY = .5;

  // Arm feedforward controller
  private final ArmFeedforward feedforward = new ArmFeedforward(A, G, S, V);

  // Trapezoidal motion profile constraints and instance
  private final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  // Spark motor controller instance
  private final SparkBase motor;
  private final RelativeEncoder encoder;

  private CommandXboxController xbox;

  /** Constructs a new ArmSubsystem by configuring the leader and follower motors. */
  public ArmSubsystem(CommandXboxController controller) {
    xbox = controller;
    File baseDirectory = new File(Filesystem.getDeployDirectory(), "motors");
    File directory = new File(baseDirectory, "arm");

    try {
      ConfiguredMotor configuredMotor =
          new MotorParser(directory).withMotor("motor.json").withPidf("pidf.json").configure();

      motor = configuredMotor.getSpark();
      encoder = motor.getEncoder();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure arm motor(s): ", exception);
    }

    ShuffleboardTab sensors = Shuffleboard.getTab("Arm Sensors");

    sensors.addDouble("Output Current", motor::getOutputCurrent);
    sensors.addDouble("Applied Output", motor::getAppliedOutput);
    sensors.addDouble("Position", encoder::getPosition);
    sensors.addDouble("Radians", this::getAngleRadians);
    sensors.addDouble("Setpoint State", () -> setpoint.position);
    sensors.addDouble("Goal State", () -> goal.position);
    sensors.addDouble("input", this::getInput);
  }

  private double getAngleRadians() {
    // arm pointed down is - 1/2 PI
    // arm pointed at horizonal is 0
    final double horizontal = 12.7;
    final double divisor = horizontal / (Math.PI / 2.0);

    return (encoder.getPosition() - horizontal) / divisor;
  }

  /**
   * Sets the target arm position and updates the motor controller reference.
   *
   * @param position The desired arm position.
   */
  private void set(ArmPosition position) {
    goal = new TrapezoidProfile.State(position.getPosition(), 0);
  }

  public double getInput() {
    return (xbox.getLeftY() * xbox.getLeftY() * Math.signum(xbox.getLeftY())) / 20.0;
  }

  @Override
  public void periodic() {

    // motor.set(getInput());

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
            feedforward.calculateWithVelocities(
                getAngleRadians(), setpoint.velocity, nextSetpoint.velocity));

    setpoint = nextSetpoint;
  }

  /**
   * Creates a command to run the arm to a specified position.
   *
   * @param position The target arm position.
   * @return A command that moves the arm.
   */
  public Command runArmTo(ArmPosition position) {
    return runOnce(() -> set(position));
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor());
  }

  /** Enum representing preset arm positions. */
  public enum ArmPosition {
    BARGE(15),
    REEF(5),
    HOME(0);

    private final double position;

    ArmPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}
