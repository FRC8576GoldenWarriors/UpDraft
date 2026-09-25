// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  private final SwerveIO io;

  private final SwerveIOInputsAutoLogged swerveInputs = new SwerveIOInputsAutoLogged();

  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final ModuleIOInputsAutoLogged frontLeftModuleInputs = new ModuleIOInputsAutoLogged();
  private final ModuleIOInputsAutoLogged frontRightModuleInputs = new ModuleIOInputsAutoLogged();
  private final ModuleIOInputsAutoLogged backLeftModuleInputs = new ModuleIOInputsAutoLogged();
  private final ModuleIOInputsAutoLogged backRightModuleInputs = new ModuleIOInputsAutoLogged();

  private final ModuleIOInputsAutoLogged[] moduleInputs = {
    frontLeftModuleInputs, frontRightModuleInputs, backLeftModuleInputs, backRightModuleInputs
  };

  public enum WantedState {
    TELEOP,
    ROTATION_LOCK,
    WHEEL_LOCK_WITH_X,
    IDLE,
    TAXI,
    SET_MODULE_ROTATIONS,
    SYS_ID_TRANSLATION,
    SYS_ID_STEER,
    SYS_ID_ROTATION,
    WHEEL_RADIUS_CHARACTERIZATION;
  }

  private enum SystemState {
    TELOP_DRIVING,
    ROTATION_LOCKING,
    WHEEL_LOCKING_WITH_X,
    IDLING,
    TAXIING,
    SETTING_MODULE_ROTATIONS,
    SYS_ID
  }

  private double xController = 0.0;

  private double yController = 0.0;

  private double omegaController = 0.0;

  private ChassisSpeeds wantedChassisSpeeds = SwerveConstants.ZERO_ROBOT_CHASSIS_SPEEDS;

  private Rotation2d wantedRotationLockRotation = new Rotation2d();

  private Rotation2d[] wantedAbsoluteModuleRotations = new Rotation2d[4];

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyFieldSpeeds teleopRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDesaturateWheelSpeeds(SwerveConstants.DESATURATE_WHEEL_SPEEDS)
          .withDriveRequestType(SwerveConstants.DRIVE_REQUEST_TYPE)
          .withSteerRequestType(SwerveConstants.STEER_REQUEST_TYPE);

  private final SwerveRequest.FieldCentricFacingAngle rotationLockRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDesaturateWheelSpeeds(SwerveConstants.DESATURATE_WHEEL_SPEEDS)
          .withDriveRequestType(SwerveConstants.DRIVE_REQUEST_TYPE)
          .withSteerRequestType(SwerveConstants.STEER_REQUEST_TYPE);

  private final SwerveRequest.SwerveDriveBrake wheelLockWithXRequest =
      new SwerveRequest.SwerveDriveBrake()
          .withDriveRequestType(SwerveConstants.DRIVE_REQUEST_TYPE)
          .withSteerRequestType(SwerveConstants.STEER_REQUEST_TYPE);

  private final SetModuleRotationsAbsolute absoluteModuleRotationRequest =
      new SetModuleRotationsAbsolute();

  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final SysIdRoutine m_sysIdRoutineTranslation;

  private final SysIdRoutine m_sysIdRoutineSteer;

  private final SysIdRoutine m_sysIdRoutineRotation;

  public Swerve(SwerveIO io) {
    this.io = io;

    rotationLockRequest.HeadingController =
        new PhoenixPIDController(
            SwerveConstants.ROTATION_KP, SwerveConstants.ROTATION_KI, SwerveConstants.ROTATION_KD);

    rotationLockRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    this.m_sysIdRoutineTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                SwerveConstants.SYS_ID_TRANSLATION_RAMP_RATE,
                SwerveConstants.SYS_ID_TRANSLATION_DYNAMIC_STEP,
                SwerveConstants.SYS_ID_TRANSLATION_TIMEOUT,
                state ->
                    Logger.recordOutput(
                        SwerveConstants.LOG_PATH + "SysId/SysIdTranslation_State",
                        state.toString())),
            new SysIdRoutine.Mechanism(
                output -> io.setSwerveState(m_translationCharacterization.withVolts(output)),
                null,
                this));

    this.m_sysIdRoutineSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                SwerveConstants.SYS_ID_STEER_RAMP_RATE,
                SwerveConstants.SYS_ID_STEER_DYNAMIC_STEP,
                SwerveConstants.SYS_ID_STEER_TIMEOUT,
                state ->
                    Logger.recordOutput(
                        SwerveConstants.LOG_PATH + "SysId/SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> io.setSwerveState(m_steerCharacterization.withVolts(volts)), null, this));

    this.m_sysIdRoutineRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                SwerveConstants.SYS_ID_ROTATION_RAMP_RATE,
                SwerveConstants.SYS_ID_ROTATION_DYNAMIC_STEP,
                SwerveConstants.SYS_ID_ROTATION_TIMEOUT,
                state ->
                    Logger.recordOutput(
                        SwerveConstants.LOG_PATH + "SysId/SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> {
                  io.setSwerveState(
                      m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                  Logger.recordOutput(
                      SwerveConstants.LOG_PATH + "SysId/Rotational_Rate", output.in(Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {

    // TODO: Check if this is actually doing what it's meant to do. SysID forward moves opposite the
    // teleop forward direction
    applyOperatorForwardPerspective();

    io.updateInputs(swerveInputs, gyroInputs, moduleInputs);

    Logger.processInputs(SwerveConstants.LOG_PATH + "Swerve", swerveInputs);

    Logger.processInputs(SwerveConstants.LOG_PATH + "Gyro", gyroInputs);

    for (int i = 0; i < moduleInputs.length; i++) {
      Logger.processInputs(
          SwerveConstants.LOG_PATH + SwerveConstants.MODULE_NAMES[i] + "Module", moduleInputs[i]);
    }

    systemState = handleStateTransition();

    Logger.recordOutput(SwerveConstants.LOG_PATH + "WantedState", wantedState);
    Logger.recordOutput(SwerveConstants.LOG_PATH + "SystemState", systemState);

    applyStates();
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case TELEOP -> SystemState.TELOP_DRIVING;

      case ROTATION_LOCK -> SystemState.ROTATION_LOCKING;

      case WHEEL_LOCK_WITH_X -> SystemState.WHEEL_LOCKING_WITH_X;

      case TAXI -> SystemState.TAXIING;

      case SET_MODULE_ROTATIONS -> {
        if (areModulesAtRotation()) {
          setWantedState(WantedState.IDLE);
          yield SystemState.IDLING;
        }

        yield SystemState.SETTING_MODULE_ROTATIONS;
      }

      case SYS_ID_TRANSLATION, SYS_ID_STEER, SYS_ID_ROTATION, WHEEL_RADIUS_CHARACTERIZATION ->
          SystemState.SYS_ID;

      case IDLE -> SystemState.IDLING;

      default -> SystemState.IDLING;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case TELOP_DRIVING -> teleopDriving();

      case ROTATION_LOCKING -> rotationLocking();

      case WHEEL_LOCKING_WITH_X -> wheelLockingWithX();

      case TAXIING -> taxiing();

      case SETTING_MODULE_ROTATIONS -> settingModuleRotations();

      case SYS_ID -> {}

      case IDLING -> idling();

      default -> idling();
    }
  }

  private void applyOperatorForwardPerspective() {
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                io.setOperatorPerspectiveForward(
                    (allianceColor == Alliance.Red)
                        ? SwerveConstants.RED_PERSPECTIVE_ROTATION
                        : SwerveConstants.BLUE_PERSPECTIVE_ROTATION);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  // Apply States Methods
  private void teleopDriving() {
    wantedChassisSpeeds =
        getChassisSpeedsFromControllerInput(xController, yController, omegaController);

    io.setSwerveState(teleopRequest.withSpeeds(wantedChassisSpeeds));
  }

  private void rotationLocking() {
    wantedChassisSpeeds =
        getChassisSpeedsFromControllerInput(xController, yController, omegaController);
    io.setSwerveState(
        rotationLockRequest
            .withVelocityX(wantedChassisSpeeds.vxMetersPerSecond)
            .withVelocityY(wantedChassisSpeeds.vyMetersPerSecond)
            .withTargetDirection(wantedRotationLockRotation));
  }

  private void wheelLockingWithX() {
    io.setSwerveState(wheelLockWithXRequest);
  }

  private void idling() {
    io.setSwerveState(idleRequest);
  }

  private void taxiing() {
    io.setSwerveState(teleopRequest.withSpeeds(SwerveConstants.TAXI_FIELD_CHASSIS_SPEEDS));
  }

  private void settingModuleRotations() {
    io.setSwerveState(
        absoluteModuleRotationRequest.withModuleRotations(wantedAbsoluteModuleRotations));
  }

  public void acceptControllerInput(
      double xController, double yController, double omegaController) {
    this.xController = xController;
    this.yController = yController;
    this.omegaController = omegaController;
  }

  public void setWantedRotation(Rotation2d wantedRotation) {
    this.wantedRotationLockRotation = wantedRotation;
    setWantedState(WantedState.ROTATION_LOCK);
  }

  public void setWantedAbsoluteModuleRotations(Rotation2d[] wantedAbsoluteRotations) {
    this.wantedAbsoluteModuleRotations = wantedAbsoluteRotations;
    setWantedState(WantedState.SET_MODULE_ROTATIONS);
  }

  public void zeroHeading() {
    io.resetRotation();
  }

  public boolean isSettingModulePositions() {
    return wantedState == WantedState.SET_MODULE_ROTATIONS
        || systemState == SystemState.SETTING_MODULE_ROTATIONS;
  }

  private boolean areModulesAtRotation() {
    return moduleInputs.length
        == IntStream.range(0, moduleInputs.length)
            .filter(
                index ->
                    MathUtil.isNear(
                        wantedAbsoluteModuleRotations[index].getDegrees(),
                        moduleInputs[index].steerPosition.getDegrees(),
                        SwerveConstants.SETTING_MODULE_ROTATION_TOLERANCE.in(Degrees)))
            .count();
  }

  public Command getAbsoluteModuleRotationsSettingCommand() {
    final Rotation2d[] moduleRotations =
        switch (wantedState) {
          case SYS_ID_TRANSLATION, SYS_ID_STEER -> SwerveConstants.MODULE_ROTATIONS_FOR_TRANSLATION;
          case SYS_ID_ROTATION, WHEEL_RADIUS_CHARACTERIZATION ->
              SwerveConstants.MODULE_ROTATIONS_FOR_ROTATION;
          default -> SwerveConstants.MODULE_ROTATIONS_FOR_TRANSLATION;
        };

    return Commands.sequence(
        Commands.runOnce(() -> setWantedAbsoluteModuleRotations(moduleRotations)),
        Commands.waitUntil(() -> !isSettingModulePositions()));
  }

  public Command getDynamicForwardCommand() {
    return Commands.deferredProxy(
        () ->
            switch (wantedState) {
              case SYS_ID_TRANSLATION -> m_sysIdRoutineTranslation.dynamic(Direction.kForward);
              case SYS_ID_STEER -> m_sysIdRoutineSteer.dynamic(Direction.kForward);
              case SYS_ID_ROTATION -> m_sysIdRoutineRotation.dynamic(Direction.kForward);
              default -> Commands.none();
            });
  }

  public Command getDynamicReverseCommand() {
    return Commands.deferredProxy(
        () ->
            switch (wantedState) {
              case SYS_ID_TRANSLATION -> m_sysIdRoutineTranslation.dynamic(Direction.kReverse);
              case SYS_ID_STEER -> m_sysIdRoutineSteer.dynamic(Direction.kReverse);
              case SYS_ID_ROTATION -> m_sysIdRoutineRotation.dynamic(Direction.kReverse);
              default -> Commands.none();
            });
  }

  public Command getQuasistaticForwardCommand() {
    return Commands.deferredProxy(
        () ->
            switch (wantedState) {
              case SYS_ID_TRANSLATION -> m_sysIdRoutineTranslation.quasistatic(Direction.kForward);
              case SYS_ID_STEER -> m_sysIdRoutineSteer.quasistatic(Direction.kForward);
              case SYS_ID_ROTATION -> m_sysIdRoutineRotation.quasistatic(Direction.kForward);
              default -> Commands.none();
            });
  }

  public Command getQuasistaticReverseCommand() {
    return Commands.deferredProxy(
        () ->
            switch (wantedState) {
              case SYS_ID_TRANSLATION -> m_sysIdRoutineTranslation.quasistatic(Direction.kReverse);
              case SYS_ID_STEER -> m_sysIdRoutineSteer.quasistatic(Direction.kReverse);
              case SYS_ID_ROTATION -> m_sysIdRoutineRotation.quasistatic(Direction.kReverse);
              default -> Commands.none();
            });
  }

  public Command getWheelRadiusCharacterizationCommand() {
    SlewRateLimiter limiter = new SlewRateLimiter(SwerveConstants.WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(SwerveConstants.WHEEL_RADIUS_MAX_VELOCITY);
                  io.setSwerveState(teleopRequest.withSpeeds(new ChassisSpeeds(0.0, 0.0, speed)));
                },
                this)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = new double[4];

                  for (int i = 0; i < state.positions.length; i++) {
                    state.positions[i] =
                        moduleInputs[i]
                            .drivePositionRad
                            .times(TunerConstants.FrontLeft.DriveMotorGearRatio)
                            .in(Radians);
                  }

                  state.lastAngle = swerveInputs.RawHeading;
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = swerveInputs.RawHeading;
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = new double[4];

                      for (int i = 0; i < positions.length; i++) {
                        positions[i] =
                            moduleInputs[i]
                                .drivePositionRad
                                .times(TunerConstants.FrontLeft.DriveMotorGearRatio)
                                .in(Radians);
                      }

                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * SwerveConstants.DRIVE_BASE_RADIUS.in(Meters))
                              / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      Logger.recordOutput(
                          SwerveConstants.LOG_PATH + "SysId/WheelDelta",
                          "Wheel Delta: " + formatter.format(wheelDelta) + " radians");
                      Logger.recordOutput(
                          SwerveConstants.LOG_PATH + "SysId/GyroDelta",
                          "Gyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      Logger.recordOutput(
                          SwerveConstants.LOG_PATH + "SysId/WheelRadiusMeters",
                          "Wheel Radius: " + formatter.format(wheelRadius) + " meters");
                      Logger.recordOutput(
                          SwerveConstants.LOG_PATH + "SysId/WheelRadiusInches",
                          "Wheel Radius: "
                              + formatter.format(Meters.of(wheelRadius).in(Inches))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  private ChassisSpeeds getChassisSpeedsFromControllerInput(
      double xController, double yController, double omegaController) {
    double xMagnitude = MathUtil.applyDeadband(xController, SwerveConstants.DEADBAND);
    double yMagnitude = MathUtil.applyDeadband(yController, SwerveConstants.DEADBAND);
    double omegaMagnitude = MathUtil.applyDeadband(omegaController, SwerveConstants.DEADBAND);

    if (xMagnitude + yMagnitude + omegaMagnitude == 0) {
      return SwerveConstants.ZERO_ROBOT_CHASSIS_SPEEDS;
    }

    if (SwerveConstants.SQUARE_INPUTS) {
      xMagnitude = MathUtil.copyDirectionPow(xMagnitude, SwerveConstants.SQUARE_VALUE);
      yMagnitude = MathUtil.copyDirectionPow(yMagnitude, SwerveConstants.SQUARE_VALUE);
      omegaMagnitude = MathUtil.copyDirectionPow(omegaMagnitude, SwerveConstants.SQUARE_VALUE);
    }

    final LinearVelocity xVelocity = TunerConstants.kSpeedAt12Volts.times(xMagnitude);
    final LinearVelocity yVelocity = TunerConstants.kSpeedAt12Volts.times(yMagnitude);
    final AngularVelocity omegaVelocity =
        SwerveConstants.MAX_ANGULAR_VELOCITY.times(omegaMagnitude);

    return new ChassisSpeeds(xVelocity, yVelocity, omegaVelocity);
  }
}
