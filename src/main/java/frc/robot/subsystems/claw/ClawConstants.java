package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Voltage;

public class ClawConstants {
  public static final String LOG_PATH = "Claw/";
  public static final String TUNING_PATH = LOG_PATH + "Roller";
  public static final boolean TUNING_MODE_ENABLED = true;

  public static final int ROLLER_MOTOR_ID = 45;

  public static final int CORAL_DIGITAL_INPUT_ID = 0;
  public static final int LEFT_REEF_DIGITAL_INPUT_ID = 1;
  public static final int RIGHT_REEF_DIGITAL_INPUT_ID = 2;

  // ! CURRENT LIMIT NOT CONFIGURED YET

  public static final Current ROLLER_STATOR_CURRENT_LIMIT = Amps.of(30);
  public static final boolean ROLLER_STATOR_CURRENT_LIMIT_ENABLED = false;
  public static final Current ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(30);
  public static final boolean ROLLER_SUPPLY_CURRENT_LIMIT_ENABLED = false;

  public static final InvertedValue ROLLER_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue ROLLER_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;

  public static final Voltage PEAK_FORWARD_VOLTAGE = Volts.of(12);
  public static final Voltage PEAK_REVERSE_VOLTAGE = Volts.of(-12);

  public static final Frequency STATUS_SIGNAL_UPDATE_FREQUENCY = Hertz.of(50);

  public static final double SCORE_CORAL_DUTY_CYCLE = 0;
  public static final double SCORE_BARGE_ALAGE_DUTY_CYCLE = 0;
  public static final double SCORE_PROCESSOR_ALAGE_DUTY_CYCLE = 0;

  public static final double PICKUP_CORAL_DUTY_CYCLE = 0;
  public static final double PICKUP_FLOOR_ALAGE_DUTY_CYCLE = 0;
  public static final double PICKUP_REEF_ALAGE_DUTY_CYCLE = 0;

  public static final double ROLLER_SENSOR_TO_MECHANISM_RATION = (36 / 12) * (48 / 8);

  public static final TalonFXConfiguration ROLLER_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(ROLLER_STATOR_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(ROLLER_STATOR_CURRENT_LIMIT_ENABLED)
                  .withSupplyCurrentLimit(ROLLER_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(ROLLER_SUPPLY_CURRENT_LIMIT_ENABLED))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(ROLLER_INVERTED_VALUE)
                  .withNeutralMode(ROLLER_NEUTRAL_MODE_VALUE))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(PEAK_FORWARD_VOLTAGE)
                  .withPeakReverseVoltage(PEAK_REVERSE_VOLTAGE));
}
