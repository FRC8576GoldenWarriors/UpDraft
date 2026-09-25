package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.util.StatusSignalRefresher;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SwerveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements SwerveIO {

  private final Consumer<GyroIOInputs> gyroInputUpdater;

  private final HashMap<Integer, Consumer<ModuleIOInputs>> moduleInputUpdater;

  private final Alert pigeonAlert;

  private final Alert[] driveMotorAlerts;

  private final Alert[] steerMotorAlerts;

  private final Alert[] canCoderAlerts;

  private final Field2d fieldViz;

  public SwerveIOCTRE(
      SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        constants,
        SwerveConstants.ODOMETRY_FREQUENCY.in(Hertz),
        SwerveConstants.DRIVETRAIN_BASE_STANDARD_DEVIATIONS,
        SwerveConstants.VISION_BASE_STANDARD_DEVIATIONS,
        moduleConstants);

    fieldViz = new Field2d();

    var statusSignalRefresher = StatusSignalRefresher.getInstance();

    var pigeon = getPigeon2();

    Supplier<Boolean> pigeonConnectedSupplier = () -> pigeon.isConnected();

    var yawSignal = pigeon.getYaw();
    var rollSignal = pigeon.getRoll();
    var pitchSignal = pigeon.getPitch();

    var yawVelocitySignal = pigeon.getAngularVelocityZWorld();
    var rollVelocitySignal = pigeon.getAngularVelocityXWorld();
    var pitchVelocitySignal = pigeon.getAngularVelocityYWorld();

    var xAccelerationSignal = pigeon.getAccelerationX();
    var yAccelerationSignal = pigeon.getAccelerationY();
    var zAccelerationSignal = pigeon.getAccelerationZ();

    pigeonAlert = new Alert("The Pigeon is disconnected", AlertType.kError);

    gyroInputUpdater =
        (GyroIOInputs gyroInputs) -> {
          gyroInputs.pigeonConnected =
              pigeonConnectedSupplier.get() && yawSignal.getStatus().isOK();
          pigeonAlert.set(!gyroInputs.pigeonConnected);

          gyroInputs.yaw = yawSignal.getValue();
          gyroInputs.roll = rollSignal.getValue();
          gyroInputs.pitch = pitchSignal.getValue();
          gyroInputs.robotOrientation =
              new Rotation3d(gyroInputs.roll, gyroInputs.pitch, gyroInputs.yaw);

          gyroInputs.yawVelocity = yawVelocitySignal.getValue();
          gyroInputs.rollVelocity = rollVelocitySignal.getValue();
          gyroInputs.pitchVelocity = pitchVelocitySignal.getValue();

          gyroInputs.xAcceleration = xAccelerationSignal.getValue();
          gyroInputs.yAcceleration = yAccelerationSignal.getValue();
          gyroInputs.zAcceleration = zAccelerationSignal.getValue();
        };

    BaseStatusSignal.setUpdateFrequencyForAll(
        SwerveConstants.FAST_UPDATE_FREQUENCY,
        yawSignal,
        rollSignal,
        pitchSignal,
        yawVelocitySignal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        SwerveConstants.SLOW_UPDATE_FREQUENCY,
        rollVelocitySignal,
        pitchVelocitySignal,
        xAccelerationSignal,
        yAccelerationSignal,
        zAccelerationSignal);

    pigeon.optimizeBusUtilization(SwerveConstants.OPTIMIZED_UPDATE_FREQUENCY);

    statusSignalRefresher.addStatusSignals(
        yawSignal,
        rollSignal,
        pitchSignal,
        yawVelocitySignal,
        rollVelocitySignal,
        pitchVelocitySignal,
        xAccelerationSignal,
        yAccelerationSignal,
        zAccelerationSignal);

    moduleInputUpdater = new HashMap<>();

    final int moduleCount = getModules().length;

    driveMotorAlerts = new Alert[moduleCount];
    steerMotorAlerts = new Alert[moduleCount];
    canCoderAlerts = new Alert[moduleCount];

    for (int i = 0; i < moduleCount; i++) {
      final int moduleIndex = i;

      final var module = getModule(i);

      final TalonFX driveMotor = module.getDriveMotor();
      final TalonFX steerMotor = module.getSteerMotor();
      final CANcoder steerCANCoder = module.getEncoder();

      driveMotorAlerts[moduleIndex] =
          new Alert(
              "The " + SwerveConstants.MODULE_NAMES[i] + " Drive motor is disconnected.",
              AlertType.kError);
      steerMotorAlerts[moduleIndex] =
          new Alert(
              "The " + SwerveConstants.MODULE_NAMES[i] + " Steer motor is disconnected.",
              AlertType.kError);
      canCoderAlerts[moduleIndex] =
          new Alert(
              "The " + SwerveConstants.MODULE_NAMES[i] + " CANCoder is disconnected.",
              AlertType.kError);

      Supplier<Boolean> driveConnectedSupplier = () -> driveMotor.isConnected();
      var drivePositionSignal = driveMotor.getPosition();
      var driveVelocitySignal = driveMotor.getVelocity();
      var driveAppliedVoltsSignal = driveMotor.getMotorVoltage();
      var driveSupplyCurrentSignal = driveMotor.getSupplyCurrent();
      var driveStatorCurrentSignal = driveMotor.getStatorCurrent();
      var driveTemperatureSignal = driveMotor.getDeviceTemp();

      Supplier<Boolean> steerConnectedSupplier = () -> steerMotor.isConnected();
      var steerPositionSignal = steerMotor.getPosition();
      var steerVelocitySignal = steerMotor.getVelocity();
      var steerAppliedVoltsSignal = steerMotor.getMotorVoltage();
      var steerSupplyCurrentSignal = steerMotor.getSupplyCurrent();
      var steerStatorCurrentSignal = steerMotor.getStatorCurrent();
      var steerTemperatureSignal = steerMotor.getDeviceTemp();

      Supplier<Boolean> canCoderConnectedSupplier = () -> steerCANCoder.isConnected();
      var steerAbsolutePositionSignal = steerCANCoder.getAbsolutePosition();
      var canCoderSteerPositionRads = steerCANCoder.getPosition();

      Consumer<ModuleIOInputs> moduleInput =
          (ModuleIOInputs moduleInputs) -> {
            // Update Drive Motor Inputs
            moduleInputs.driveConnected =
                driveConnectedSupplier.get() && drivePositionSignal.getStatus().isOK();
            moduleInputs.drivePositionRad = drivePositionSignal.getValue();
            moduleInputs.driveVelocityRadPerSec = driveVelocitySignal.getValue();
            moduleInputs.driveAppliedVolts = driveAppliedVoltsSignal.getValue();
            moduleInputs.driveSupplyCurrentAmps = driveSupplyCurrentSignal.getValue();
            moduleInputs.driveStatorCurrentAmps = driveStatorCurrentSignal.getValue();
            moduleInputs.driveTemperatureCelsius = driveTemperatureSignal.getValue();
            // Update Steer Motor Inputs
            moduleInputs.steerConnected =
                steerConnectedSupplier.get() && steerPositionSignal.getStatus().isOK();
            moduleInputs.steerPositionRads = steerPositionSignal.getValue();
            moduleInputs.steerPosition = new Rotation2d(moduleInputs.steerPositionRads);
            moduleInputs.steerVelocityRadPerSec = steerVelocitySignal.getValue();
            moduleInputs.steerAppliedVolts = steerAppliedVoltsSignal.getValue();
            moduleInputs.steerSupplyCurrentAmps = steerSupplyCurrentSignal.getValue();
            moduleInputs.steerStatorCurrentAmps = steerStatorCurrentSignal.getValue();
            moduleInputs.steerTemperatureCelsius = steerTemperatureSignal.getValue();

            moduleInputs.canCoderConnected = canCoderConnectedSupplier.get();
            moduleInputs.canCoderSteerPositionRads = canCoderSteerPositionRads.getValue();
            moduleInputs.steerAbsolutePosition =
                new Rotation2d(steerAbsolutePositionSignal.getValue());

            // Update the Alerts
            driveMotorAlerts[moduleIndex].set(!moduleInputs.driveConnected);
            steerMotorAlerts[moduleIndex].set(!moduleInputs.steerConnected);
            canCoderAlerts[moduleIndex].set(!moduleInputs.canCoderConnected);
          };

      moduleInputUpdater.put(i, moduleInput);

      BaseStatusSignal.setUpdateFrequencyForAll(
          SwerveConstants.FAST_UPDATE_FREQUENCY,
          drivePositionSignal,
          driveVelocitySignal,
          driveAppliedVoltsSignal,
          steerPositionSignal,
          steerVelocitySignal,
          steerAppliedVoltsSignal,
          steerAbsolutePositionSignal,
          canCoderSteerPositionRads,
          driveStatorCurrentSignal);

      BaseStatusSignal.setUpdateFrequencyForAll(
          SwerveConstants.SLOW_UPDATE_FREQUENCY,
          driveSupplyCurrentSignal,
          driveTemperatureSignal,
          steerSupplyCurrentSignal,
          steerStatorCurrentSignal,
          steerTemperatureSignal);

      ParentDevice.optimizeBusUtilizationForAll(
          SwerveConstants.OPTIMIZED_UPDATE_FREQUENCY, driveMotor, steerMotor, steerCANCoder);

      statusSignalRefresher.addStatusSignals(
          drivePositionSignal,
          driveVelocitySignal,
          driveAppliedVoltsSignal,
          driveSupplyCurrentSignal,
          driveStatorCurrentSignal,
          driveTemperatureSignal,
          steerPositionSignal,
          steerVelocitySignal,
          steerAppliedVoltsSignal,
          steerSupplyCurrentSignal,
          steerStatorCurrentSignal,
          steerTemperatureSignal,
          steerAbsolutePositionSignal,
          canCoderSteerPositionRads);
    }
  }

  @Override
  public void updateInputs(
      SwerveIOInputs swerveInputs, GyroIOInputs gyroInputs, ModuleIOInputs... moduleInputs) {
    swerveInputs.fromSwerveState(this.getState());

    fieldViz.setRobotPose(swerveInputs.Pose);

    gyroInputUpdater.accept(gyroInputs);

    // TODO: Check if its worth it to transfer robot state heading to gyro yaw

    final SwerveModuleState[] currentModuleStates = swerveInputs.ModuleStates;
    final SwerveModuleState[] targetModuleStates = swerveInputs.ModuleTargets;
    final SwerveModulePosition[] currentSwerveModulePosition = swerveInputs.ModulePositions;

    final int moduleCount = moduleInputs.length;
    for (int i = 0; i < moduleCount; i++) {
      var moduleInput = moduleInputs[i];
      moduleInputUpdater.get(i).accept(moduleInput);

      moduleInput.currentState = currentModuleStates[i];
      moduleInput.targetState = targetModuleStates[i];
      moduleInput.currentSwerveModulePosition = currentSwerveModulePosition[i];
    }
  }

  @Override
  public void setSwerveState(SwerveRequest swerveRequest) {
    super.setControl(swerveRequest);
  }

  @Override
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return getKinematics();
  }

  @Override
  public void resetOdometry(Pose2d pose2d) {
    super.resetPose(pose2d);
  }

  @Override
  public void resetRotation() {
    super.resetRotation(getOperatorForwardDirection());
  }

  @Override
  public void resetRotation(Rotation2d rotation) {
    super.resetRotation(rotation);
  }

  @Override
  public void resetTranslation() {
    super.resetTranslation(Translation2d.kZero);
  }

  @Override
  public void resetTranslation(Translation2d translation) {
    super.resetTranslation(translation);
  }

  @Override
  public void setOperatorPerspectiveForward(Rotation2d rotation2d) {
    super.setOperatorPerspectiveForward(rotation2d);
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    super.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  @Override
  public void setDrivetrainStandardDeviations(Matrix<N3, N1> standardDeviations) {
    super.setStateStdDevs(standardDeviations);
  }

  @Override
  public void setVisionStandardDeviations(Matrix<N3, N1> standardDeviations) {
    super.setVisionMeasurementStdDevs(standardDeviations);
  }
}
