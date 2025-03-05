package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Radians;

import bearlib.motor.deserializer.MotorParser;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

public class ArmSubsystem extends SubsystemBase {
  // Feedforward gains and motion parameters
  private final double A = 0;
  private final double S = 0.010;
  private final double G = 0.0095; // / or maybe .0145???
  private final double V = 0;

  public final double MAX_ACCELERATION = 150;
  public final double MAX_VELOCITY = 200; // higher than what we need

  // arm pointed down is - 1/2 PI
  // arm pointed at horizonal is 0
  public final double HORIZONTAL = 9.52;
  public final double ANGLE_DIVISOR = HORIZONTAL / (Math.PI / 2);

  // Spark motor controller instance
  @Logged(name = "Arm Motor")
  private final SparkBase motor;

  @Logged(name = "Arm Encoder")
  private final RelativeEncoder encoder;

  // Arm feedforward controller
  private final ArmFeedforward feedforward = new ArmFeedforward(A, G, S, V);

  // Trapezoidal motion profile constraints and instance
  private final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);

  @Logged(name = "Arm Goal")
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  @Logged(name = "Arm Setpoint")
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  /** Constructs a new ArmSubsystem by configuring the leader and follower motors. */
  public ArmSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "motors/arm");

    try {
      motor =
          new MotorParser(directory).withMotor("motor.json").withPidf("pidf.json").configureAsync();
      encoder = motor.getEncoder();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure arm motor(s): ", exception);
    }
  }

  @Logged(name = "Arm At Setpoint")
  public boolean isAtSetpoint() {
    return trapezoidProfile.timeLeftUntil(goal.position) == 0;
  }

  private Angle getAngleRadians() {
    return Radians.of((encoder.getPosition() - HORIZONTAL) / ANGLE_DIVISOR);
  }

  /**
   * Sets the target arm position and updates the motor controller reference.
   *
   * @param position The desired arm position.
   */
  private void set(ArmPosition position) {
    goal = new TrapezoidProfile.State(position.getPosition(), 0);
  }

  public void set(double speed) {
    motor.set(speed);
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
            feedforward.calculateWithVelocities(
                getAngleRadians().magnitude(), setpoint.velocity, nextSetpoint.velocity));

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
    REEF(3),
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
