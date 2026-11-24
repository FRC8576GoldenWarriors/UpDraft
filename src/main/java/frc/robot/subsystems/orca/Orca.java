// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.orca;

import frc.robot.RobotContainer;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.loggedimpl.LoggedSubsystem;

public class Orca extends LoggedSubsystem {
  /** Creates a new Orca. */
  private final RobotContainer robotContainer;

  private final Swerve swerve;
  private final Intake intake;
  private final Transport transport;
  private final Cradle cradle;

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
    WHEEL_RADIUS_CHARACTERIZATION,
    HOME_INTAKE,
    ZERO,
    INTAKE,
    RETRACT,
    OUTTAKE,
    HOLD
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
    WHEEL_RADIUS_CHARACTERIZATION,
    HOMING_INTAKE,
    ZEROING,
    INTAKING,
    RETRACTING,
    OUTTAKING,
    HOLDING
  }

  private WantedState wantedState = WantedState.IDLE;

  private SystemState systemState = SystemState.IDLE;

  public Orca(RobotContainer robotContainer) {
    super(OrcaConstants.LOG_PATH, null, true);
    this.robotContainer = robotContainer;
    this.swerve = robotContainer.getSwerveSubsystem();
    this.intake = robotContainer.getIntakeSubsystem();
    this.transport = robotContainer.getTransportSubsystem();
    this.cradle = robotContainer.getCradleSubsystem();
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

    record("WantedState", wantedState);
    record("CurrentState", systemState);

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
      case HOME_INTAKE -> SystemState.HOMING_INTAKE;
      case ZERO -> SystemState.ZEROING;
      case INTAKE -> SystemState.INTAKING;
      case RETRACT -> SystemState.RETRACTING;
      case OUTTAKE -> SystemState.OUTTAKING;
      case HOLD -> SystemState.HOLDING;
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
      case HOMING_INTAKE -> home();
      case ZEROING -> zeroing();
      case INTAKING -> intaking();
      case RETRACTING -> retracting();
      case OUTTAKING -> outtaking();
      case HOLDING -> holding();
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
    intake.setWantedState(Intake.WantedState.IDLE);
    transport.setWantedState(Transport.WantedState.IDLE);
  }

  private void zeroing() {
    intake.setWantedState(Intake.WantedState.ZERO);
  }

  private void home() {
    intake.setWantedState(Intake.WantedState.HOME);
  }

  private void setpoint() {
    intake.setWantedState(Intake.WantedState.POSITION_MANUALLY);
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

  private void intaking() {
    swerve.setWantedState(Swerve.WantedState.TELEOP);
    intake.setWantedState(Intake.WantedState.INTAKE);
    transport.setWantedState(Transport.WantedState.TRANSPORT_IN);
    if (cradle.isHoldingPartially()) {
      setWantedState(WantedState.HOLD);
    }
  }

  private void holding() {
    swerve.setWantedState(Swerve.WantedState.TELEOP);
    intake.setWantedState(Intake.WantedState.RETRACT);
    transport.setWantedState(Transport.WantedState.IDLE);
  }

  private void retracting() {
    swerve.setWantedState(Swerve.WantedState.TELEOP);
    intake.setWantedState(Intake.WantedState.RETRACT);
    transport.setWantedState(Transport.WantedState.IDLE);
  }

  private void outtaking() {
    swerve.setWantedState(Swerve.WantedState.TELEOP);
    intake.setWantedState(Intake.WantedState.RETRACT);
    transport.setWantedState(Transport.WantedState.TRANSPORT_OUT);
  }
}
