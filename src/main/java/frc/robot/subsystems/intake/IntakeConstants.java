package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.configs.IntakeConfig;

public class IntakeConstants {

  public static final String LOG_PATH = "Intake/";

  private static final int INTAKE_PIVOT_MOTOR_ID = 1;
  private static final int INTAKE_ROLLER_MOTOR_ID = 2;

  private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
  private static final Angle GRAVITY_ARM_POSITION_OFFSET = Radians.of(0);
  private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE =
      StaticFeedforwardSignValue.UseVelocitySign;

  private static final double KP = 0.0;
  private static final double KI = 0.0;
  private static final double KD = 0.0;
  private static final double KS = 0.0;
  private static final double KG = 0.0;
  private static final double KV = 0.0;

  // ! CURRENT LIMIT NOT CONFIGURED YET
  private static final Current PIVOT_STATOR_CURRENT_LIMIT = Amps.of(30);
  private static final boolean PIVOT_STATOR_CURRENT_LIMIT_ENABLED = false;
  private static final Current PIVOT_SUPPLY_CURRENT_LIMIT = Amps.of(60);
  private static final boolean PIVOT_SUPPLY_CURRENT_LIMIT_ENABLED = false;

  private static final InvertedValue PIVOT_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
  private static final NeutralModeValue PIVOT_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;

  private static final AngularVelocity MOTION_MAGIC_ANGULAR_VELOCITY = RadiansPerSecond.of(15);
  private static final AngularAcceleration MOTION_MAGIC_ANGULAR_ACCELERATION =
      RadiansPerSecondPerSecond.of(60);

  private static final Current ROLLER_STATOR_CURRENT_LIMIT = Amps.of(30);
  private static final boolean ROLLER_STATOR_CURRENT_LIMIT_ENABLED = false;
  private static final Current ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(30);
  private static final boolean ROLLER_SUPPLY_CURRENT_LIMIT_ENABLED = false;

  private static final InvertedValue ROLLER_INVERTED_VALUE =
      InvertedValue.CounterClockwise_Positive;
  private static final NeutralModeValue ROLLER_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;

  private static final Voltage PEAK_FORWARD_VOLTAGE = Volts.of(12);
  private static final Voltage PEAK_REVERSE_VOLTAGE = Volts.of(-12);

  private static final Frequency STATUS_SIGNAL_UPDATE_FREQUENCY = Hertz.of(50);

  private static final Current PIVOT_HOMING_CURRENT = Amps.of(4);

  private static final Angle INTAKE_DOWN_POSITION = Radians.of(0.5);
  private static final Angle INTAKE_UP_POSITION = Radians.of(0.5);

  public static final IntakeConfig INTAKE_CONFIG =
      new IntakeConfig()
          .withIntakePivotId(INTAKE_PIVOT_MOTOR_ID)
          .withIntakeRollerId(INTAKE_ROLLER_MOTOR_ID)
          .withIntakePivotConfig(
              new TalonFXConfiguration()
                  .withSlot0(
                      new Slot0Configs()
                          .withGravityType(GRAVITY_TYPE_VALUE)
                          .withGravityArmPositionOffset(GRAVITY_ARM_POSITION_OFFSET)
                          .withStaticFeedforwardSign(STATIC_FEEDFORWARD_SIGN_VALUE)
                          .withKP(KP)
                          .withKI(KI)
                          .withKD(KD)
                          .withKS(KS)
                          .withKG(KG)
                          .withKS(KV))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(PIVOT_STATOR_CURRENT_LIMIT)
                          .withStatorCurrentLimitEnable(PIVOT_STATOR_CURRENT_LIMIT_ENABLED)
                          .withSupplyCurrentLimit(PIVOT_SUPPLY_CURRENT_LIMIT)
                          .withSupplyCurrentLimitEnable(PIVOT_SUPPLY_CURRENT_LIMIT_ENABLED))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(PIVOT_INVERTED_VALUE)
                          .withNeutralMode(PIVOT_NEUTRAL_MODE_VALUE))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicCruiseVelocity(MOTION_MAGIC_ANGULAR_VELOCITY)
                          .withMotionMagicAcceleration(MOTION_MAGIC_ANGULAR_ACCELERATION)))
          .withIntakeRollerConfig(
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
                          .withPeakReverseVoltage(PEAK_REVERSE_VOLTAGE)))
          .withIntakeStatusSignalUpdateFrequency(STATUS_SIGNAL_UPDATE_FREQUENCY);
}
