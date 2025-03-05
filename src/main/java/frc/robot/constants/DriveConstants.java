package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  public static final int DRIVER_JOYSTICK_PORT = 0;
  public static final int OPERATOR_JOYSTICK_PORT = 1;
  public static final int OPERATOR_GAMEPAD_PORT = 2;

  public static double MAX_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static double MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  public static final double LINEAR_DEADBAND = MAX_VELOCITY * 0.1; // 10% deadband
  public static final double ROTATIONAL_DEADBAND = MAX_ANGULAR_VELOCITY * 0.1; // 10% deadband

  // Field centric swerve request
  public static final SwerveRequest.FieldCentric FIELD_CENTRIC_SWERVE_REQUEST =
      new SwerveRequest.FieldCentric()
          .withDeadband(LINEAR_DEADBAND)
          .withRotationalDeadband(ROTATIONAL_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public static final Telemetry TELEMETRY = new Telemetry(MAX_VELOCITY);
}
