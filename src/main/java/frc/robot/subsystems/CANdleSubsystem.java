package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  private final CANdle CANdle = new CANdle(43);

  public int[] getRgbFromColor(Color color) {
    return new int[] {(int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)};
  }

  /**
   * Set the color of the LEDs
   *
   * @param color The color of the LEDs
   * @return THe command to change the color.
   */
  public Command setColor(Color color) {
    int[] rgb = getRgbFromColor(color);

    Animation animation = new LarsonAnimation(rgb[0], rgb[1], rgb[2]);
    animation.setLedOffset(0);
    animation.setSpeed(0.1);

    return runOnce(() -> CANdle.setLEDs(rgb[0], rgb[1], rgb[2]))
        .andThen(runOnce(() -> CANdle.animate(animation)));
  }

  public Command setAnimation(Animation animation) {
    return runOnce(() -> CANdle.animate(animation));
  }
}
