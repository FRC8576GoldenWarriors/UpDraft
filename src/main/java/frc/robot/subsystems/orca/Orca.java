// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.orca;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;

public class Orca extends SubsystemBase {
  /** Creates a new Orca. */
  private final RobotContainer robotContainer;

  private final Swerve swerve;

  private Pose2d currentRobotPose = new Pose2d();

  private ChassisSpeeds currentChassisSpeeds = SwerveConstants.ZERO_ROBOT_CHASSIS_SPEEDS;
  private LinearVelocity chassisSpeedMagnitude = MetersPerSecond.zero();

  public enum WantedState {
    // Swerve States
    DEFAULT_STATE,
    TELEOP,
    ROTATION_LOCK,
    WHEEL_LOCK_WITH_X,
    IDLE,
    TAXI,
    SYS_ID_TRANSLATION,
    SYS_ID_STEER,
    SYS_ID_ROTATION,
    WHEEL_RADIUS_CHARACTERIZATION;
  }

  private enum SystemState {
    // Swerve States
    DEFAULT_STATE,
    TELEOP,
    ROTATION_LOCK,
    WHEEL_LOCK_WITH_X,
    IDLE,
    TAXI,
    SYS_ID_TRANSLATION,
    SYS_ID_STEER,
    SYS_ID_ROTATION,
    WHEEL_RADIUS_CHARACTERIZATION;
  }

  private WantedState wantedState = WantedState.IDLE;

  private SystemState systemState = SystemState.IDLE;

  public Orca(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.swerve = robotContainer.getSwerveSubsystem();
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public WantedState getWantedState() {
    return this.wantedState;
  }

  @Override
  public void periodic() {

    // ! Add a method to track robot state. Should include pose, chassisSpeed, and other important
    // state data.

    systemState = handleStateTransitions();

    Logger.recordOutput(OrcaConstants.LOG_PATH + "WantedState", wantedState);
    Logger.recordOutput(OrcaConstants.LOG_PATH + "CurrentState", systemState);

    applyStates();
  }

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case TELEOP -> SystemState.TELEOP;
      case ROTATION_LOCK -> SystemState.ROTATION_LOCK;
      case WHEEL_LOCK_WITH_X -> SystemState.WHEEL_LOCK_WITH_X;
      case IDLE -> SystemState.IDLE;
      case TAXI -> SystemState.TAXI;
      case SYS_ID_TRANSLATION -> SystemState.SYS_ID_TRANSLATION;
      case SYS_ID_STEER -> SystemState.SYS_ID_STEER;
      case SYS_ID_ROTATION -> SystemState.SYS_ID_ROTATION;
      case WHEEL_RADIUS_CHARACTERIZATION -> SystemState.WHEEL_RADIUS_CHARACTERIZATION;
      default -> SystemState.IDLE;
    };
  }

  private void applyStates() {

    switch (systemState) {
      case TELEOP -> teleop();
      case ROTATION_LOCK -> rotationLock();
      case WHEEL_LOCK_WITH_X -> wheelLockWithX();
      case IDLE -> idling();
      case TAXI -> taxi();
      case SYS_ID_TRANSLATION -> sysIdTranslation();
      case SYS_ID_STEER -> sysIdSteer();
      case SYS_ID_ROTATION -> sysIdRotation();
      case WHEEL_RADIUS_CHARACTERIZATION -> wheelRadiusCharacterization();
      default -> idling();
    }
  }

  private void teleop() {
    swerve.setWantedState(Swerve.WantedState.TELEOP);
  }

  private void rotationLock() {
    swerve.setWantedState(Swerve.WantedState.ROTATION_LOCK);
  }

  private void wheelLockWithX() {
    swerve.setWantedState(Swerve.WantedState.WHEEL_LOCK_WITH_X);
  }

  private void idling() {
    swerve.setWantedState(Swerve.WantedState.IDLE);
  }

  private void taxi() {
    swerve.setWantedState(Swerve.WantedState.TAXI);
  }

  private void sysIdTranslation() {
    swerve.setWantedState(Swerve.WantedState.SYS_ID_TRANSLATION);
  }

  private void sysIdSteer() {
    swerve.setWantedState(Swerve.WantedState.SYS_ID_STEER);
  }

  private void sysIdRotation() {
    swerve.setWantedState(Swerve.WantedState.SYS_ID_ROTATION);
  }

  private void wheelRadiusCharacterization() {
    swerve.setWantedState(Swerve.WantedState.WHEEL_RADIUS_CHARACTERIZATION);
  }
}
