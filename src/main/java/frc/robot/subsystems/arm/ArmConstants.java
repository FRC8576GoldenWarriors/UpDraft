package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import frc.robot.util.PIDFConstants;

public class ArmConstants {
  public static final String LOG_PATH = "Arm/";
  public static final String PIVOT_TUNING_PATH = "Pivot";

  public static final boolean TUNING_MODE_ENABLED = true;

  public static final int PIVOT_MOTOR = 60;

  public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
  public static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE =
      StaticFeedforwardSignValue.UseVelocitySign;

  public static final double KP = 0.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KS = 0.0;
  public static final double KG = 0.0;
  public static final double KV = 0.0;
  public static final double KA = 0.0;

  public static final PIDFConstants PIDF_CONSTANTS = new PIDFConstants(KP, KI, KD, KS, KG, KV, KA);

  public static final Current PIVOT_STATOR_CURRENT_LIMIT = Amps.of(0);
  public static final boolean PIVOT_STATOR_CURRENT_LIMIT_ENABLED = false;

  public static final Current PIVOT_SUPPLY_CURRENT_LIMIT = Amps.of(0);
  public static final boolean PIVOT_SUPPLY_CURRENT_LIMIT_ENABLED = false;

  public static final InvertedValue PIVOT_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue PIVOT_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;

  public static final AngularVelocity PIVOT_MOTION_MAGIC_ANGULAR_VELOCITY =
      RotationsPerSecond.of(8);
  public static final AngularAcceleration PIVOT_MOTION_MAGIC_ANGULAR_ACCELERATION =
      RotationsPerSecondPerSecond.of(8);

  public static final double PIVOT_SENSOR_TO_MECHANISM_RATIO = 69;

  public static final Frequency PIVOT_STATUS_SIGNAL_UPDATE_FREQUENCY = Hertz.of(50);

  public static final double PIVOT_HOMING_DUTY_CYCLE = 0.15;
  public static final Current PIVOT_HOMING_CURRENT = Amps.of(89);
  public static final Current PIVOT_HOMING_CURRENT_TOLERANCE = Amps.of(2);
  public static final AngularVelocity PIVOT_HOMING_VELOCITY = RotationsPerSecond.of(0);
  public static final AngularVelocity PIVOT_HOMING_VELOCITY_TOLERANCE = RotationsPerSecond.of(0.1);

  public static final Angle PIVOT_POSITION_TOLERANCE = Rotations.of(0.005);

  public static final Angle PIVOT_MAX_ANGLE = Rotations.of(0);
  public static final Angle PIVOT_EXPECTED_HOMING_ZERO = Rotations.of(0);
  public static final Angle PIVOT_MIN_ANGLE = Rotations.of(0);

  public static final Angle PICKUP_CORAL_POSITION = Rotations.of(0);
  public static final Angle CORAL_L1_POSITION = Rotations.of(0);
  public static final Angle CORAL_L2_POSITION = Rotations.of(0);
  public static final Angle CORAL_L3_POSITION = Rotations.of(0);
  public static final Angle CORAL_L4_POSITION = Rotations.of(0);

  public static final Angle ALAGE_L1_POSITION = Rotations.of(0);
  public static final Angle ALAGE_L2_POSITION = Rotations.of(0);
  public static final Angle ALAGE_FLOOR_POSITION = Rotations.of(0);
  public static final Angle ALAGE_LOLIPOP_POSITION = Rotations.of(0);

  public static final Angle BARGE_POSITION = Rotations.of(0);
  public static final Angle STANDBY_POSITION = Rotations.of(0);

  public static final TalonFXConfiguration PIVOT_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(
              PIDFConstants.toSlot0Configs(PIDF_CONSTANTS)
                  .withGravityType(GRAVITY_TYPE_VALUE)
                  .withStaticFeedforwardSign(STATIC_FEEDFORWARD_SIGN_VALUE))
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
                  .withMotionMagicCruiseVelocity(PIVOT_MOTION_MAGIC_ANGULAR_VELOCITY)
                  .withMotionMagicAcceleration(PIVOT_MOTION_MAGIC_ANGULAR_ACCELERATION))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(PIVOT_SENSOR_TO_MECHANISM_RATIO));
}
