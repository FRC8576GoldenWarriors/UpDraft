package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {

  @AutoLog
  class SwerveIOInputs extends SwerveDriveState {

    void fromSwerveState(SwerveDriveState state) {
      this.FailedDaqs = state.FailedDaqs;
      this.ModulePositions = state.ModulePositions;
      this.ModuleStates = state.ModuleStates;
      this.ModuleTargets = state.ModuleTargets;
      this.OdometryPeriod = state.OdometryPeriod;
      this.Pose = state.Pose;
      this.RawHeading = state.RawHeading;
      this.Speeds = state.Speeds;
      this.SuccessfulDaqs = state.SuccessfulDaqs;
      this.Timestamp = state.Timestamp;
    }
  }

  @AutoLog
  class GyroIOInputs {
    // Connection
    public boolean pigeonConnected = false;
    // Angle Inputs
    public Angle yaw = Radians.of(0.0);
    public Angle roll = Radians.of(0.0);
    public Angle pitch = Radians.of(0.0);
    public Rotation3d robotOrientation = Rotation3d.kZero;

    // Velocity Inputs
    public AngularVelocity yawVelocity = RadiansPerSecond.of(0.0);
    public AngularVelocity rollVelocity = RadiansPerSecond.of(0.0);
    public AngularVelocity pitchVelocity = RadiansPerSecond.of(0.0);

    // Acceleration Inputs
    public LinearAcceleration xAcceleration = MetersPerSecondPerSecond.of(0.0);
    public LinearAcceleration yAcceleration = MetersPerSecondPerSecond.of(0.0);
    public LinearAcceleration zAcceleration = MetersPerSecondPerSecond.of(0.0);
  }

  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public Angle drivePositionRad = Radians.of(0.0);
    public AngularVelocity driveVelocityRadPerSec = RadiansPerSecond.of(0.0);
    public Voltage driveAppliedVolts = Volts.of(0.0);
    public Current driveSupplyCurrentAmps = Amps.of(0.0);
    public Current driveStatorCurrentAmps = Amps.of(0.0);
    public Temperature driveTemperatureCelsius = Celsius.of(0.0);

    public boolean steerConnected = false;
    public Angle steerPositionRads = Radians.of(0.0);
    public Rotation2d steerPosition = Rotation2d.kZero;
    public AngularVelocity steerVelocityRadPerSec = RadiansPerSecond.of(0.0);
    public Voltage steerAppliedVolts = Volts.of(0.0);
    public Current steerSupplyCurrentAmps = Amps.of(0.0);
    public Current steerStatorCurrentAmps = Amps.of(0.0);
    public Temperature steerTemperatureCelsius = Celsius.of(0.0);

    public boolean canCoderConnected = false;
    public Angle canCoderSteerPositionRads = Radians.of(0);
    public Rotation2d steerAbsolutePosition = Rotation2d.kZero;

    public SwerveModuleState currentState = new SwerveModuleState();
    public SwerveModuleState targetState = new SwerveModuleState();
    public SwerveModulePosition currentSwerveModulePosition = new SwerveModulePosition();
  }

  void updateInputs(
      SwerveIOInputs swerveInputs, GyroIOInputs gyroInputs, ModuleIOInputs... moduleInputs);

  SwerveDriveKinematics getSwerveDriveKinematics();

  default void setSwerveState(SwerveRequest swerveRequest) {}

  void resetOdometry(Pose2d pose2d);

  default void resetRotation() {}

  default void resetToRotation(Rotation2d rotation) {}

  default void resetTranslation() {}

  default void setOperatorPerspectiveForward(Rotation2d rotation2d) {}

  default void addVisionMeasurement(Pose2d pose, double timestamp) {}

  default void setDrivetrainStandardDeviations(Matrix<N3, N1> standardDeviations) {}

  default void setVisionStandardDeviations(Matrix<N3, N1> standardDeviations) {}
}
