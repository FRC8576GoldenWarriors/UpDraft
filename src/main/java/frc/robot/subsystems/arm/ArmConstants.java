package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ArmConstants {
  public static final String LOG_PATH = "Arm/";
  public static final boolean TUNING_MODE_ENABLED = true;

  public static final int PIVOT_MOTOR = 60;

  public static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
  public static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE =
      StaticFeedforwardSignValue.UseVelocitySign;
}
