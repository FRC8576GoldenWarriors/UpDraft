package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {
  public static final String LOG_PATH = "Elevator";
  public static final boolean TUNING_MODE_ENABLED = true;

  public static final int LEFT_LIFT_MOTOR_ID = 35;
  public static final int RIGHT_LIFT_MOTOR_ID = 36;

  public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;
  public static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE =
      StaticFeedforwardSignValue.UseVelocitySign;

  public static final double KP = 2.551;
  public static final double KI = 0.0;
  public static final double KD = 0.066;
  public static final double KS = 0.0;
  public static final double KG = 0.391;
  public static final double KV = 0.118;
  public static final double KA = 0.002;

  // ! CURRENT LIMIT NOT CONFIGURED YET
  public static final Current LIFT_STATOR_CURRENT_LIMIT = Amps.of(110);
  public static final boolean LIFT_STATOR_CURRENT_LIMIT_ENABLED = true;
  public static final Current LIFT_SUPPLY_CURRENT_LIMIT = Amps.of(50);
  public static final boolean LIFT_SUPPLY_CURRENT_LIMIT_ENABLED = true;

  public static final InvertedValue LEFT_LIFT_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue LEFT_LIFT_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

  public static final InvertedValue RIGHT_LIFT_INVERTED_VALUE =
      InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue RIGHT_LIFT_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

  public static final LinearVelocity MOTION_MAGIC_ANGULAR_VELOCITY = MetersPerSecond.of(4.753);
  public static final LinearAcceleration MOTION_MAGIC_ANGULAR_ACCELERATION =
      MetersPerSecondPerSecond.of(57.031);

  public static final Frequency STATUS_SIGNAL_UPDATE_FREQUENCY = Hertz.of(50);

  public static final Voltage LIFT_HOMING_VOLTAGE = Volts.of(-2);

  public static final Current LIFT_HOMING_CURRENT = Amps.of(100);
  public static final Current LIFT_HOMING_CURRENT_TOLERANCE = Amps.of(2);
  public static final LinearVelocity LIFT_HOMING_VELOCITY = MetersPerSecond.of(0);
  public static final LinearVelocity LIFT_HOMING_VELOCITY_TOLERANCE = MetersPerSecond.of(0.05);

  public static final Distance LIFT_POSITION_TOLERANCE = Meters.of(0.005);
  public static final Distance PITCH_DIAMETER = Meters.of(0.057);

  public static final double LIFT_SENSOR_TO_MECHANISM_RATIO = (60 / 18);

  public static final Dimensionless LINEAR_TO_ANGULAR =
      Value.of(LIFT_SENSOR_TO_MECHANISM_RATIO).div(Value.of(Math.PI * PITCH_DIAMETER.in(Meters)));

  public static final TalonFXConfiguration LEFT_LIFT_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withGravityType(GRAVITY_TYPE_VALUE)
                  .withStaticFeedforwardSign(STATIC_FEEDFORWARD_SIGN_VALUE)
                  .withKP(KP)
                  .withKI(KI)
                  .withKD(KD)
                  .withKS(KS)
                  .withKG(KG)
                  .withKS(KV)
                  .withKA(KA))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(LIFT_STATOR_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(LIFT_STATOR_CURRENT_LIMIT_ENABLED)
                  .withSupplyCurrentLimit(LIFT_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(LIFT_SUPPLY_CURRENT_LIMIT_ENABLED))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(LEFT_LIFT_INVERTED_VALUE)
                  .withNeutralMode(LEFT_LIFT_NEUTRAL_MODE_VALUE))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(
                      MOTION_MAGIC_ANGULAR_VELOCITY.in(MetersPerSecond)
                          * LINEAR_TO_ANGULAR.in(Value))
                  .withMotionMagicAcceleration(
                      MOTION_MAGIC_ANGULAR_ACCELERATION.in(MetersPerSecondPerSecond)
                          * LINEAR_TO_ANGULAR.in(Value)))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(LIFT_SENSOR_TO_MECHANISM_RATIO));

  public static final TalonFXConfiguration RIGHT_LIFT_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withGravityType(GRAVITY_TYPE_VALUE)
                  .withStaticFeedforwardSign(STATIC_FEEDFORWARD_SIGN_VALUE)
                  .withKP(KP)
                  .withKI(KI)
                  .withKD(KD)
                  .withKS(KS)
                  .withKG(KG)
                  .withKS(KV)
                  .withKA(KA))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(LIFT_STATOR_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(LIFT_STATOR_CURRENT_LIMIT_ENABLED)
                  .withSupplyCurrentLimit(LIFT_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(LIFT_SUPPLY_CURRENT_LIMIT_ENABLED))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(RIGHT_LIFT_INVERTED_VALUE)
                  .withNeutralMode(RIGHT_LIFT_NEUTRAL_MODE_VALUE))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(
                      MOTION_MAGIC_ANGULAR_VELOCITY.in(MetersPerSecond)
                          * LINEAR_TO_ANGULAR.in(Value))
                  .withMotionMagicAcceleration(
                      MOTION_MAGIC_ANGULAR_ACCELERATION.in(MetersPerSecondPerSecond)
                          * LINEAR_TO_ANGULAR.in(Value)))
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(LIFT_SENSOR_TO_MECHANISM_RATIO));

  public static Distance angularToLinear(Angle angle) {
    return Meters.of(angle.in(Rotations) * LINEAR_TO_ANGULAR.in(Value));
  }

  public static Angle linearToAngular(Distance distance) {
    return Rotations.of(distance.in(Meters) / LINEAR_TO_ANGULAR.in(Value));
  }

  public static LinearVelocity angularToLinear(AngularVelocity angularVelocity) {
    return MetersPerSecond.of(angularVelocity.in(RotationsPerSecond) * LINEAR_TO_ANGULAR.in(Value));
  }

  public static AngularVelocity linearToAngular(LinearVelocity linearVelocity) {
    return RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / LINEAR_TO_ANGULAR.in(Value));
  }

  public static LinearAcceleration angularToLinear(AngularAcceleration angularAcceleration) {
    return MetersPerSecondPerSecond.of(
        angularAcceleration.in(RotationsPerSecondPerSecond) * LINEAR_TO_ANGULAR.in(Value));
  }

  public static AngularAcceleration linearToAngular(LinearAcceleration linearAcceleration) {
    return RotationsPerSecondPerSecond.of(
        linearAcceleration.in(MetersPerSecondPerSecond) / LINEAR_TO_ANGULAR.in(Value));
  }
}
