package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A class that sets the colors on the LED strip. */
public class LEDs extends SubsystemBase {
  /**
   * The object that communicates with the LED driver, which is a device that connects the RoboRIO's
   * PWM port to the LEDs.
   */
  private AddressableLED addressableLeds;

  /** The buffer that holds the color values for each LED. */
  private AddressableLEDBuffer colorBuffer;

  /**
   * Creates a continuous, scrolling gradient pattern between two colors. *
   *
   * <p>This method generates a smooth, continuous transition from the starting color to the ending
   * color and animates it moving across the LED strip at the specified rate.
   */
  private static LEDPattern createContinuousGradient(Color from, Color to, Frequency scrollSpeed) {
    return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, from, to)
        .scrollAtRelativeSpeed(scrollSpeed);
  }

  /** Sets the color yellow for the hopper full state. */
  public Command hopperFull() {
    return runEnd(
            () -> {
              for (int i = 0; i < colorBuffer.getLength(); i++) {
                colorBuffer.setRGB(i, 255, 255, 0);
              }
            },
            () -> {
              for (int i = 0; i < colorBuffer.getLength(); i++) {
                colorBuffer.setRGB(i, 0, 0, 0);
              }
            })
        .withTimeout(2.0); // T.B.D
  }

  /** Sets the color purple for the hopper empty state. */
  public Command hopperEmpty() {
    return runEnd(
            () -> {
              for (int i = 0; i < colorBuffer.getLength(); i++) {
                colorBuffer.setRGB(i, 255, 0, 255);
              }
            },
            () -> {
              for (int i = 0; i < colorBuffer.getLength(); i++) {
                colorBuffer.setRGB(i, 0, 0, 0);
              }
            })
        .withTimeout(2.0); // T.B.D
  }

  /** Sets the gradient blue for the teleop state. */
  public Command teleopBlue() {
    return run(
        () -> {
          Color lightBlue = new Color(0, 100, 255);
          Color darkBlue = new Color(0, 0, 50);
          LEDPattern pattern = createContinuousGradient(lightBlue, darkBlue, Hertz.of(1)); // T.B.D
          pattern.applyTo(colorBuffer);
          addressableLeds.setData(colorBuffer);
        });
  }

  /** Sets the gradient red for the teleop state. */
  public Command teleopRed() {
    return run(
        () -> {
          Color lightRed = new Color(255, 0, 0);
          Color darkRed = new Color(50, 0, 0);
          LEDPattern pattern = createContinuousGradient(lightRed, darkRed, Hertz.of(1)); // T.B.D
          pattern.applyTo(colorBuffer);
          addressableLeds.setData(colorBuffer);
        });
  }

  /** Sets the rainbow pattern for the pose indicator. */
  public Command rainbowPose() {
    return run(
        () -> {
          LEDPattern pattern =
              LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(1)); // T.B.D
          pattern.applyTo(colorBuffer);
          addressableLeds.setData(colorBuffer);
        });
  }
}
