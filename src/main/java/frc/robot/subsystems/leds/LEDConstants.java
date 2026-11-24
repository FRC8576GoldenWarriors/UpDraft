package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {

  public static final String LOG_PATH = "LEDs/";

  public static final int PORT = 1;
  public static final int STRIP_LENGTH = 60;

  public static final int SECTION_AMOUNT = 1;
  public static final Distance SPACING = Meters.of(1 / STRIP_LENGTH);

  class Patterns {
    public static final LEDPattern IDLE =
        LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kGold)
            .scrollAtRelativeSpeed(Percent.per(Seconds).of(10));
  }
}
