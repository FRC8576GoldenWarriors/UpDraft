// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.loggedimpl.LoggedSysIdRoutine;
import frc.robot.util.loggedimpl.LoggedSystemState;
import frc.robot.util.loggedimpl.LoggedWantedState;
import frc.robot.util.loggedtunable.LoggedTunableDouble;

public class Intake extends IntakeStateMachine {

  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final LoggedTunableDouble manualPositionSetpoint;

  private boolean isHomed = false;

  public final LoggedSysIdRoutine pivotRoutine =
      this.newSubsystemLoggedSysIdRoutine(Volts.per(Seconds).of(1), Volts.of(4), Seconds.of(4));

  public enum WantedState implements LoggedWantedState {
    IDLE,
    DEPLOY,
    RETRACT,
    INTAKE,
    HOME,
    POSITION_MANUALLY,
    ZERO
  }

  private enum SystemState implements LoggedSystemState {
    IDLING,
    DEPLOYING,
    RETRACTING,
    INTAKING,
    HOMING,
    POSITIONING_MANUALLY,
    ZEROING
  }

  public Intake(IntakeIO io) {
    super(IntakeConstants.LOG_PATH, io.getCurrentSignals());
    this.io = io;
    manualPositionSetpoint =
        newSubsystemLoggedTunableDouble(
            "ManualSetpoint", (value) -> {}, IntakeConstants.TUNING_MODE_ENABLED);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    processInputs(inputs);

    systemState = handleStateTransition();
    recordWantedState(wantedState);
    recordSystemState(systemState);
    record("IsHomed", isHomed);

    applyStates();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLING;
      case DEPLOY -> SystemState.DEPLOYING;
      case RETRACT -> SystemState.RETRACTING;
      case INTAKE -> SystemState.INTAKING;
      case HOME -> {
        if (systemState != SystemState.HOMING) {
          isHomed = false;
        }
        yield SystemState.HOMING;
      }
      case POSITION_MANUALLY -> SystemState.POSITIONING_MANUALLY;
      case ZERO -> SystemState.ZEROING;
      default -> SystemState.IDLING;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLING -> idling();
      case DEPLOYING -> deploying();
      case RETRACTING -> retracting();
      case INTAKING -> intaking();
      case HOMING -> homing();
      case POSITIONING_MANUALLY -> positioningManually();
      case ZEROING -> zeroing();
      default -> idling();
    }
  }

  @Override
  protected void idling() {
    io.idle();
  }

  @Override
  protected void deploying() {
    io.setWantedPosition(IntakeConstants.DOWN_POSITION);
    io.setWantedRollerSpeed(IntakeConstants.DEPLOYING_DUTY_CYCLE);
  }

  @Override
  protected void retracting() {
    io.setWantedPosition(IntakeConstants.RETRACT_POSITION);
    io.setWantedRollerSpeed(IntakeConstants.RETRACTING_DUTY_CYCLE);
  }

  @Override
  protected void intaking() {
    io.setWantedPosition(IntakeConstants.DOWN_POSITION);
    io.setWantedRollerSpeed(IntakeConstants.INTAKING_DUTY_CYCLE);
  }

  @Override
  protected void positioningManually() {
    io.setWantedPosition(Rotations.of(manualPositionSetpoint.get()));
    io.setWantedRollerSpeed(0);
  }

  protected void zeroing() {
    io.zero();
    setWantedState(WantedState.IDLE);
  }

  protected void voltageIn(Voltage voltageIn) {
    io.setVoltage(voltageIn);
  }

  @Override
  protected void homing() {
    this.isHomed = io.home();
    if (isHomed) {
      setWantedState(WantedState.IDLE);
    }
  }

  public boolean nearSetpoint() {
    return io.nearSetpoint();
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return this.wantedState;
  }

  public boolean isIntakeHomed() {
    return isHomed;
  }
}
