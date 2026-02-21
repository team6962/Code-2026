package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  /** This constructor initializes the LED subsystem. */
  public LEDs() {
    this.addressableLeds = new AddressableLED(LEDsConstants.pwmPort);
    this.colorBuffer = new AddressableLEDBuffer(LEDsConstants.length);
  }

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

  /** Clears all LEDs by setting them to black. */
  public void clearLeds() {
    for (int i = 0; i < colorBuffer.getLength(); i++) {
      colorBuffer.setRGB(i, 0, 0, 0);
    }
    addressableLeds.setData(colorBuffer);
  }

  /** Sets the color yellow for the hopper full state. */
  public Command hopperFull() {
    return runEnd(
            () -> {
              for (int i = 0; i < colorBuffer.getLength(); i++) {
                colorBuffer.setRGB(i, 204, 255, 0);
              }
              addressableLeds.setData(colorBuffer);
            },
            () -> {
              clearLeds();
            })
        .withTimeout(2.0);
  }

  /** Sets the color purple for the hopper empty state. */
  public Command hopperEmpty() {
    return runEnd(
            () -> {
              for (int i = 0; i < colorBuffer.getLength(); i++) {
                colorBuffer.setRGB(i, 255, 0, 255);
              }
              addressableLeds.setData(colorBuffer);
            },
            () -> {
              clearLeds();
            })
        .withTimeout(2.0);
  }

  /** Sets the gradient blue for the teleop state. */
  public Command teleopBlue() {
    return runEnd(
        () -> {
          Color blue = new Color(0, 0, 255);
          Color cyan = new Color(0, 255, 255);
          LEDPattern pattern = createContinuousGradient(blue, cyan, Hertz.of(1)); // T.B.D
          pattern.applyTo(colorBuffer);
          addressableLeds.setData(colorBuffer);
        },
        () -> {
          clearLeds();
        });
  }

  /** Sets the gradient red for the teleop state. */
  public Command teleopRed() {
    return runEnd(
        () -> {
          Color red = new Color(255, 0, 0);
          Color pink = new Color(255, 0, 255);
          LEDPattern pattern = createContinuousGradient(red, pink, Hertz.of(1)); // T.B.D
          pattern.applyTo(colorBuffer);
          addressableLeds.setData(colorBuffer);
        },
        () -> {
          clearLeds();
        });
  }

  /** Sets the rainbow pattern for the pose indicator. */
  public Command rainbowPose() {
    return runEnd(
        () -> {
          LEDPattern pattern =
              LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(1)); // T.B.D
          pattern.applyTo(colorBuffer);
          addressableLeds.setData(colorBuffer);
        },
        () -> {
          clearLeds();
        });
  }

  /* Sets the gradient green and pink for Auton state.*/
  public Command autonColors() {
    return Commands.either(
        runEnd(
            () -> {
              Color yellow = new Color(204, 255, 0);
              Color blue = new Color(0, 255, 255);
              LEDPattern pattern = createContinuousGradient(blue, yellow, Hertz.of(1)); // T.B.D
              pattern.applyTo(colorBuffer);
              addressableLeds.setData(colorBuffer);
            },
            () -> {
              clearLeds();
            }),
        runEnd(
            () -> {
              Color yellow = new Color(204, 255, 0);
              Color red = new Color(255, 0, 0);
              LEDPattern pattern = createContinuousGradient(red, yellow, Hertz.of(1)); // T.B.D
              pattern.applyTo(colorBuffer);
              addressableLeds.setData(colorBuffer);
            },
            () -> {
              clearLeds();
            }),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue));
  }
}
