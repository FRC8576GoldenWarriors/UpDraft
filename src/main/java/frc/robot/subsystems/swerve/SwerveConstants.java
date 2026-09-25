package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.orca.Orca.WantedState;

public class SwerveConstants {

  // ==========================================
  // Telemetry & Update Frequencies
  // ==========================================
  public static final String LOG_PATH = "Drive/";
  public static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

  public static final Frequency FAST_UPDATE_FREQUENCY = Hertz.of(100);
  public static final Frequency SLOW_UPDATE_FREQUENCY = Hertz.of(0);
  public static final Frequency OPTIMIZED_UPDATE_FREQUENCY = Hertz.of(0);

  // ==========================================
  // Kinematics & Physical Limits
  // ==========================================
  public static final Distance DRIVE_BASE_RADIUS =
      Meters.of(Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY));

  public static final AngularVelocity MAX_ANGULAR_VELOCITY =
      RadiansPerSecond.of(
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));

  public static final ChassisSpeeds ZERO_ROBOT_CHASSIS_SPEEDS = new ChassisSpeeds();
  public static final ChassisSpeeds TAXI_FIELD_CHASSIS_SPEEDS = new ChassisSpeeds(0.5, 0, 0);

  public static final Rotation2d[] MODULE_ROTATIONS_FOR_TRANSLATION = {
    Rotation2d.kZero, Rotation2d.kZero, Rotation2d.k180deg, Rotation2d.k180deg
  };

  public static final Rotation2d[] MODULE_ROTATIONS_FOR_ROTATION = {
    Rotation2d.fromDegrees(135),
    Rotation2d.fromDegrees(45),
    Rotation2d.fromDegrees(-135),
    Rotation2d.fromDegrees(-45)
  };
  public static final Angle SETTING_MODULE_ROTATION_TOLERANCE = Degrees.of(1);

  public static final Rotation2d BLUE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
  public static final Rotation2d RED_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

  // ==========================================
  // Drive & Steer Request Configurations
  // ==========================================
  public static final boolean DESATURATE_WHEEL_SPEEDS = true;
  public static final DriveRequestType DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;
  public static final SteerRequestType STEER_REQUEST_TYPE = SteerRequestType.Position;

  // ==========================================
  // Drive and Steer PIDF Gains
  // ==========================================

  // Steer Feedforward
  public static final double STEER_KS = 0.1;
  public static final double STEER_KV = 2.49;
  public static final double STEER_KA = 0.0;

  // Steer PID
  public static final double STEER_KP = 100;
  public static final double STEER_KI = 0.0;
  public static final double STEER_KD = 0.5;

  // Drive Feedforward
  public static final double DRIVE_KS = 0;
  public static final double DRIVE_KV = 0.124;
  public static final double DRIVE_KA = 0;

  // Drive PID
  public static final double DRIVE_KP = 0.1;
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KD = 0;

  // Heading PID
  public static final double ROTATION_KP = 5;
  public static final double ROTATION_KI = 0;
  public static final double ROTATION_KD = 0.15;

  // ==========================================
  // Driver Input & Deadband Settings
  // ==========================================
  public static final double DEADBAND = 0.05;
  public static final boolean SQUARE_INPUTS = true;
  public static final double SQUARE_VALUE = 2.0;

  // ==========================================
  // Odometry & Vision Pose Estimation
  // ==========================================
  public static final Frequency ODOMETRY_FREQUENCY = Hertz.of(250);

  public static final Matrix<N3, N1> DRIVETRAIN_BASE_STANDARD_DEVIATIONS =
      VecBuilder.fill(0.1, 0.1, 0.001);

  public static final Matrix<N3, N1> VISION_BASE_STANDARD_DEVIATIONS =
      VecBuilder.fill(0.25, 0.25, Integer.MAX_VALUE);

  // ==========================================
  // SysId Characterization & Tuning Constants
  // =====================================
  public static final boolean USE_SYS_ID_MODE = true;
  ;
  public static final WantedState WANTED_SYS_ID_STATE = WantedState.SYS_ID_TRANSLATION;
  public static final boolean USE_TUNING_MODE = false;

  // Translation
  public static final Velocity<VoltageUnit> SYS_ID_TRANSLATION_RAMP_RATE = Volts.of(1).per(Second);
  public static final Voltage SYS_ID_TRANSLATION_DYNAMIC_STEP = Volts.of(4);
  public static final Time SYS_ID_TRANSLATION_TIMEOUT = Seconds.of(10);

  // Steer
  public static final Velocity<VoltageUnit> SYS_ID_STEER_RAMP_RATE = Volts.of(1).per(Second);
  public static final Voltage SYS_ID_STEER_DYNAMIC_STEP = Volts.of(7);
  public static final Time SYS_ID_STEER_TIMEOUT = Seconds.of(10);

  // Rotation
  public static final Velocity<VoltageUnit> SYS_ID_ROTATION_RAMP_RATE =
      Volts.of(Math.PI / 6).per(Second);
  public static final Voltage SYS_ID_ROTATION_DYNAMIC_STEP = Volts.of(Math.PI);
  public static final Time SYS_ID_ROTATION_TIMEOUT = Seconds.of(10);

  // Wheel Radius
  public static final double WHEEL_RADIUS_RAMP_RATE = 0.05;
  public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
}
