package frc.robot.subsystems.Transport;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class transportConstants {
 public static final String LOG_PATH = "Transport/";
        public static final Frequency updateFrequency = Hertz.of(50);
    public static final Time debounceTime = Seconds.of(0.2);
  public static final int TRANSPORT_LEFT_MOTOR_ID = 0;
  public static final int TRANSPORT_RIGHT_MOTOR_ID = 0;

  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KS = 0.0;
  public static final double KG = 0.0;
  public static final double KV = 0.0;


  public static final Current TRANSPORT_SUPPLY_CURRENT_LIMIT = Amps.of(40);
  public static final boolean TRANSPORT_SUPPLY_CURRENT_LIMIT_ENABLED = true;

  public static final InvertedValue TRANSPORT_LEFT_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue TRANSPORT_RIGHT_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

  public static final NeutralModeValue TRANSPORT_RIGHT_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;

     public static final AngularVelocity transportInSpeed = RPM.of(2000);
    public static final AngularVelocity transportInSlow = RotationsPerSecond.of(75);
    public static final AngularVelocity transportOutSpeed = RPM.of(-2000);

}
