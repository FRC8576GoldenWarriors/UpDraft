// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Transport;

import frc.robot.util.loggedtunable.LoggedTunableDouble;

public class Transport extends TransportStateMachine {

  private TransportIO io;

  private TransportIOInputsAutoLogged inputs = new TransportIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final LoggedTunableDouble manualPositionSetpoint;

  private boolean isHomed = false;

  public enum WantedState {
    IDLE,
    TRANSPORT_IN,
    TRANSPORT_OUT
  }

  private enum SystemState {
    IDLING,
    TRANSPORTING_IN,
    TRANSPORTING_OUT
  }

  public Transport(TransportIO io) {
    super(transportConstants.LOG_PATH, io.getCurrentSignals());
    this.io = io;
    manualPositionSetpoint =
        newSubsystemLoggedTunableDouble("ManualSetpoint", (value) -> {}, false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    processInputs(inputs);

    systemState = handleStateTransition();
    record("WantedState", wantedState);
    record("SystemState", wantedState);
    record("IsHomed", isHomed);

    applyStates();
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
    // TODO Auto-generated method stub
    io.idle();
  }

  @Override
  protected void transportingIn() {
    io.setTransportSpeed(transportConstants.transportInSpeed, transportConstants.transportInSpeed);
    ;
  }

  @Override
  protected void transportingOut() {
    io.setTransportSpeed(
        transportConstants.transportOutSpeed, transportConstants.transportOutSpeed);
    ;
  }
}
