package frc.robot.subsystems.Transport;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public class TransportConstants {
  public static final String LOG_PATH = "Transport/";
  public static final Frequency updateFrequency = Hertz.of(50);
  public static final Time debounceTime = Seconds.of(0.2);

  public static final int TRANSPORT_LEFT_MOTOR_ID = 25;
  public static final int TRANSPORT_RIGHT_MOTOR_ID = 26;

  public static final boolean TRANSPORT_TUNING_MODE_ENABLED = false;

  public static final Current LEFT_SUPPLY_CURRENT_LIMIT = Amps.of(40);
  public static final boolean LEFT_SUPPLY_CURRENT_LIMIT_ENABLED = true;

  public static final Current RIGHT_SUPPLY_CURRENT_LIMIT = Amps.of(40);
  public static final boolean RIGHT_SUPPLY_CURRENT_LIMIT_ENABLED = true;

  public static final InvertedValue LEFT_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

  public static final InvertedValue RIGHT_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

  public static final NeutralModeValue LEFT_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
  public static final NeutralModeValue RIGHT_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(TransportConstants.LEFT_NEUTRAL_MODE_VALUE)
                  .withInverted(TransportConstants.LEFT_INVERTED_VALUE))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(TransportConstants.LEFT_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(
                      TransportConstants.LEFT_SUPPLY_CURRENT_LIMIT_ENABLED));

  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(TransportConstants.RIGHT_NEUTRAL_MODE_VALUE)
                  .withInverted(TransportConstants.RIGHT_INVERTED_VALUE))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(TransportConstants.RIGHT_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(
                      TransportConstants.RIGHT_SUPPLY_CURRENT_LIMIT_ENABLED));

  public static final double IN_DUTY_CYCLE_LEFT = 0.4;
  public static final double IN_DUTY_CYLE_RIGHT = 0.4;

  public static final double OUT_DUTY_CYCLE_LEFT = -0.6;
  public static final double OUT_DUTY_CYLE_RIGHT = -0.6;
}
