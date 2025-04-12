package frc.robot.subsystems;

import bearlib.fms.AllianceColor;
import bearlib.fms.AllianceReadyListener;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.manipulator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ElevatorSubsystem.ElevatorPosition;
import java.util.function.Supplier;

/**
 * Subsystem to control the CANdle LED strip.
 *
 * <p>This subsystem is responsible for managing solid colors, LED animations, and logic-based
 * indicators reflecting game state, alliance, and manipulator position (e.g., elevator).
 */
public class CANdleSubsystem extends SubsystemBase implements AllianceReadyListener {
  private static final int CANDLE_PORT = 43;
  private static final double DEFAULT_BRIGHTNESS = 1;

  private static final int CENTER_SCAN_START_INDEX = 21;
  private static final int CENTER_SCAN_LED_COUNT = 19;

  private static final int STATIC_WHITE_SECTION_LEFT_START = 8;
  private static final int STATIC_WHITE_SECTION_LEFT_COUNT = 12;
  private static final int STATIC_WHITE_SECTION_RIGHT_START = 42;
  private static final int STATIC_WHITE_SECTION_RIGHT_COUNT = 14;

  private static final int ELEVATOR_INDICATOR_LED_COUNT = 4;
  private static final int L4_INDICATOR_LEFT_START = 8;
  private static final int L3_INDICATOR_LEFT_START = 12;
  private static final int L2_INDICATOR_LEFT_START = 16;
  private static final int L4_INDICATOR_RIGHT_START = 50;
  private static final int L3_INDICATOR_RIGHT_START = 46;
  private static final int L2_INDICATOR_RIGHT_START = 42;

  private static final Color COLOR_CORAL_STROBE = new Color(25, 255, 25);

  private final CANdle candle;
  private final ElevatorSubsystem elevator;

  private final Supplier<ElevatorPosition> elevatorPositionSupplier;

  private boolean hasReset = false;
  private int barcodeCount;

  /** Constructs the CANdleSubsystem and applies configuration to the LED controller. */
  public CANdleSubsystem(
      ElevatorSubsystem elevator, Supplier<ElevatorPosition> elevatorPositionSupplier) {
    this.elevatorPositionSupplier = elevatorPositionSupplier;
    this.elevator = elevator;

    candle = new CANdle(CANDLE_PORT);

    CANdleConfiguration config = new CANdleConfiguration();
    config.vBatOutputMode = VBatOutputMode.Modulated;
    config.brightnessScalar = DEFAULT_BRIGHTNESS;

    ErrorCode error = candle.configAllSettings(config, 100);

    if (error != ErrorCode.OK) {
      DriverStation.reportError("Could not configure CANdle: " + error, false);
    }

    AllianceColor.addListener(this);
  }

  /**
   * Sets the entire LED strip to a solid color.
   *
   * @param color the color to apply to all LEDs
   */
  public void setColor(Color color) {
    int[] rgb = colorToRgbIntArray(color);
    candle.setLEDs(rgb[0], rgb[1], rgb[2]);
  }

  /**
   * Returns a command that sets the LED strip to a solid color.
   *
   * @param color the color to apply
   * @return a command that runs once to set the LEDs
   */
  public Command setColorCommand(Color color) {
    return runOnce(() -> setColor(color)).withName("SetCandleColor");
  }

  /**
   * Runs the given animation in slot 0.
   *
   * @param animation the animation to run
   */
  public void setAnimation(Animation animation) {
    candle.animate(animation, 0);
  }

  /**
   * Runs the given animation in the specified animation slot.
   *
   * @param animation the animation to run
   * @param slot the animation slot (0–7)
   */
  public void setAnimation(Animation animation, int slot) {
    candle.animate(animation, slot);
  }

  /**
   * Returns a command to run the given animation in slot 0.
   *
   * @param animation the animation to run
   * @return a command that runs once to activate the animation
   */
  public Command setAnimationCommand(Animation animation) {
    return runOnce(() -> setAnimation(animation)).withName("SetCandleAnimation");
  }

  /**
   * Returns a command that activates a coral-colored strobe effect across the LED strip.
   *
   * @return a command that runs once to set the strobe, wait one second, and reset.
   */
  public Command setCoralStrobeCommand() {
    final StrobeAnimation animation =
        new StrobeAnimation(
            (int) (COLOR_CORAL_STROBE.red * 255),
            (int) (COLOR_CORAL_STROBE.green * 255),
            (int) (COLOR_CORAL_STROBE.blue * 255));

    animation.setSpeed(0);

    return setAnimationCommand(animation)
        .andThen(Commands.waitSeconds(1))
        .andThen(runOnce(this::reset))
        .withName("SetCandleCoralStrobe");
  }

  /**
   * Resets the LED strip based on elevator position and current alliance.
   *
   * <p>Applies a center scan animation, static white zones, and colored indicators.
   *
   * @param elevatorPosition the current elevator state
   */
  public void reset() {
    setColor(Color.kBlack);
    clearAllAnimations();

    startCenterScanAnimation();
    setStaticWhiteIndicators();

    final ElevatorPosition elevatorPosition = elevatorPositionSupplier.get();

    if (elevatorPosition != ElevatorPosition.HOME) {
      setAllianceElevatorIndicators(elevatorPosition);
    }
  }

  /** Clears all active animations from all CANdle animation slots. */
  public void clearAllAnimations() {
    for (int i = 0; i < 8; i++) {
      candle.clearAnimation(i);
    }
  }

  /** Starts a white center Larson animation in slot 0. */
  private void startCenterScanAnimation() {
    Color color = getAllianceColor();

    LarsonAnimation animation =
        new LarsonAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            255,
            0,
            CENTER_SCAN_LED_COUNT,
            BounceMode.Center,
            4,
            CENTER_SCAN_START_INDEX);

    setAnimation(animation, 0);
  }

  /** Lights the static white segments at the outer portions of the LED strip. */
  private void setStaticWhiteIndicators() {
    setLedRange(Color.kWhite, STATIC_WHITE_SECTION_LEFT_START, STATIC_WHITE_SECTION_LEFT_COUNT);
    setLedRange(Color.kWhite, STATIC_WHITE_SECTION_RIGHT_START, STATIC_WHITE_SECTION_RIGHT_COUNT);
  }

  /**
   * Sets LED segments to indicate the current elevator level using alliance colors.
   *
   * @param elevatorPosition the current elevator level
   */
  private void setAllianceElevatorIndicators(ElevatorPosition elevatorPosition) {
    int indicator1Start = getLeftIndicatorStart(elevatorPosition);
    int indicator2Start = getRightIndicatorStart(elevatorPosition);
    Color allianceColor = getAllianceColor();

    if (indicator1Start > 0)
      setLedRange(allianceColor, indicator1Start, ELEVATOR_INDICATOR_LED_COUNT);
    if (indicator2Start > 0)
      setLedRange(allianceColor, indicator2Start, ELEVATOR_INDICATOR_LED_COUNT);
  }

  /**
   * Returns the LED start index for the left indicator corresponding to the elevator level.
   *
   * @param position the elevator level
   * @return the LED index, or 0 if invalid
   */
  private int getLeftIndicatorStart(ElevatorPosition position) {
    return switch (position) {
      case L4 -> L4_INDICATOR_LEFT_START;
      case L3 -> L3_INDICATOR_LEFT_START;
      case L2 -> L2_INDICATOR_LEFT_START;
      default -> 0;
    };
  }

  /**
   * Returns the LED start index for the right indicator corresponding to the elevator level.
   *
   * @param position the elevator level
   * @return the LED index, or 0 if invalid
   */
  private int getRightIndicatorStart(ElevatorPosition position) {
    return switch (position) {
      case L4 -> L4_INDICATOR_RIGHT_START;
      case L3 -> L3_INDICATOR_RIGHT_START;
      case L2 -> L2_INDICATOR_RIGHT_START;
      default -> 0;
    };
  }

  /**
   * Determines the current alliance color and returns a respective WPILib {@link Color}.
   *
   * @return {@link Color.kRed} or {@link Color.kBlue}, defaulting to {@link Color.kWhite}
   */
  private Color getAllianceColor() {
    Alliance alliance = AllianceColor.getAlliance();

    if (alliance == null) {
      return Color.kWhite;
    }

    return alliance == Alliance.Red ? Color.kRed : Color.kBlue;
  }

  /**
   * Converts a WPILib {@link Color} to an RGB array of ints.
   *
   * @param color the color to convert
   * @return array in the format [R, G, B]
   */
  private int[] colorToRgbIntArray(Color color) {
    return new int[] {(int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)};
  }

  /**
   * Sets a range of LEDs to a solid color.
   *
   * @param color the color to apply
   * @param startIndex the starting LED index
   * @param count the number of LEDs to set
   */
  private void setLedRange(Color color, int startIndex, int count) {
    int[] rgb = colorToRgbIntArray(color);
    candle.setLEDs(rgb[0], rgb[1], rgb[2], 0, startIndex, count);
  }

  /** Update the side LEDs based on elevator height. */
  @Override
  public void periodic() {
    boolean isAtHome =
        MathUtil.isNear(ElevatorPosition.HOME.getPosition(), elevator.getPosition(), 2);

    if (isAtHome) {
      if (!hasReset) {
        reset();
        hasReset = true;
      }
      return;
    }

    // Elevator is not at home, so allow future resets and update LED barcodes
    hasReset = false;
    applyElevatorHeightBarcode();
  }

  /** Fill in the side LEDs based on the current elevator height. */
  private void applyElevatorHeightBarcode() {
    // 3.25 found from `Elevator Encoder At L4 / LED Pixel Count`
    final int count = (int) Math.round(elevator.getPosition() / 3.25); // LED per elevator encoder
    final boolean doneAnimating = count == barcodeCount;

    if (doneAnimating) {
      return;
    } else {
      barcodeCount = count;
    }

    final int stripCount = 12;

    setLedRange(Color.kBlack, STATIC_WHITE_SECTION_LEFT_START, 12);
    setLedRange(Color.kBlack, STATIC_WHITE_SECTION_RIGHT_START, 14);

    setLedRange(getAllianceColor(), STATIC_WHITE_SECTION_LEFT_START + stripCount - count, count);
    setLedRange(getAllianceColor(), STATIC_WHITE_SECTION_RIGHT_START, count);
  }

  /** Reset the colors to the current alliance color. */
  @Override
  public void updateAlliance(Alliance alliance) {
    reset();
  }
}
