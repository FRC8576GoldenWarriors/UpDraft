// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.orca;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.loggedimpl.LoggedSubsystem;
import java.util.Arrays;

public class Orca extends LoggedSubsystem {
  /** Creates a new Orca. */
  private final RobotContainer robotContainer;

  private final Swerve swerve;
  private final Intake intake;
  private final Transport transport;
  private final Cradle cradle;
  private final Arm arm;
  private final Claw claw;
  private final Elevator elevator;

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
    HOLD,
    HOME_ARM
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
    HOLDING,
    HOMING_ARM
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
    this.claw = robotContainer.getClawSubsystem();
    this.arm = robotContainer.getArmSubsystem();
    this.elevator = robotContainer.getElevatorSubsystem();
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public Command commandWantedState(WantedState state) {
    return Commands.runOnce(() -> setWantedState(state)).withName(state.toString());
  }

  public Command commandWantedState(WantedState state, String commandName) {
    return Commands.runOnce(() -> setWantedState(state)).withName(commandName);
  }

  public Command toggleWantedState(WantedState firstToggleFirst, WantedState secondToggleSecond) {
    return Commands.either(
            commandWantedState(secondToggleSecond),
            commandWantedState(firstToggleFirst),
            () -> getWantedState() == firstToggleFirst)
        .withName("Toggle " + firstToggleFirst.toString() + " / " + secondToggleSecond.toString());
  }

  public WantedState getWantedState() {
    return this.wantedState;
  }

  public boolean stateEquals(WantedState state) {
    return getWantedState() == state;
  }

  public boolean stateEqualsAny(WantedState... states) {
    return Arrays.stream(states).anyMatch(this::stateEquals);
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
      case HOME_ARM -> SystemState.HOMING_ARM;
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
      case HOMING_INTAKE -> homeIntake();
      case HOMING_ARM -> homingArm();
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

  private void homing() {
    swerve.setWantedState(Swerve.WantedState.IDLE);
    elevator.setWantedState(Elevator.WantedState.HOME);
  }

  private void idling() {
    swerve.setWantedState(Swerve.WantedState.IDLE);
    intake.setWantedState(Intake.WantedState.IDLE);
    transport.setWantedState(Transport.WantedState.IDLE);
  }

  private void zeroing() {
    arm.setWantedState(Arm.WantedState.ZERO);
  }

  private void homeIntake() {
    intake.setWantedState(Intake.WantedState.HOME);
  }

  private void homingArm() {
    arm.setWantedState(Arm.WantedState.HOME);
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
    if (cradle.isHoldingPartially()) {
      setWantedState(WantedState.HOLD);
    }
    swerve.setWantedState(Swerve.WantedState.TELEOP);
    intake.setWantedState(Intake.WantedState.INTAKE);
    transport.setWantedState(Transport.WantedState.TRANSPORT_IN);
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
