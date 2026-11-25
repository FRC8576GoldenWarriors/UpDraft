// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import frc.robot.util.loggedimpl.LoggedSystemState;
import frc.robot.util.loggedimpl.LoggedWantedState;
import frc.robot.util.loggedtunable.LoggedTunableDouble;

public class Transport extends TransportStateMachine {

  private TransportIO io;

  private TransportIOInputsAutoLogged inputs = new TransportIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final LoggedTunableDouble manualPositionSetpoint;

  private boolean isHomed = false;

  public enum WantedState implements LoggedWantedState {
    IDLE,
    TRANSPORT_IN,
    TRANSPORT_OUT
  }

  private enum SystemState implements LoggedSystemState {
    IDLING,
    TRANSPORTING_IN,
    TRANSPORTING_OUT
  }

  public Transport(TransportIO io) {
    super(TransportConstants.LOG_PATH, io.getCurrentSignals());
    this.io = io;
    manualPositionSetpoint =
        newSubsystemLoggedTunableDouble("ManualSetpoint", (value) -> {}, false);
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

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return this.wantedState;
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLING;
      case TRANSPORT_IN -> SystemState.TRANSPORTING_IN;
      case TRANSPORT_OUT -> SystemState.TRANSPORTING_OUT;
      default -> SystemState.IDLING;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLING -> idling();
      case TRANSPORTING_IN -> transportingIn();
      case TRANSPORTING_OUT -> transportingOut();
      default -> idling();
    }
  }

  @Override
  protected void idling() {
    io.idle();
  }

  @Override
  protected void transportingIn() {
    io.setTransportPercentVoltage(
        TransportConstants.IN_DUTY_CYCLE_LEFT, TransportConstants.IN_DUTY_CYLE_RIGHT);
  }

  @Override
  protected void transportingOut() {
    io.setTransportPercentVoltage(
        TransportConstants.OUT_DUTY_CYCLE_LEFT, TransportConstants.OUT_DUTY_CYLE_RIGHT);
  }
}
