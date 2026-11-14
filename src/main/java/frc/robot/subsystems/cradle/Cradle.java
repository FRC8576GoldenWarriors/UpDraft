package frc.robot.subsystems.cradle;

import frc.robot.util.loggedimpl.LoggedSystemState;
import frc.robot.util.loggedimpl.LoggedWantedState;

public class Cradle extends CradleStateMachine {

  private final CradleIO io;
  private final CradleIOInputsAutoLogged inputs = new CradleIOInputsAutoLogged();

  public enum WantedState implements LoggedWantedState {
    HOLD_PARTIAL,
    HOLD_FULL,
    IDLE
  }

  private enum SystemState implements LoggedSystemState {
    HOLDING_PARTIALLY,
    HOLDING_FULLY,
    IDLING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  public Cradle(CradleIO io) {
    super(CradleConstants.LOG_PATH, null);
    this.io = io;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    processInputs(inputs);

    systemState = handleStateTransition();

    recordWantedState(wantedState);
    recordSystemState(systemState);
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case HOLD_PARTIAL -> SystemState.HOLDING_PARTIALLY;
      case HOLD_FULL -> SystemState.HOLDING_FULLY;
      case IDLE -> SystemState.IDLING;
      default -> SystemState.IDLING;
    };
  }

  @Override
  public boolean isHoldingPartially() {
    return inputs.backCANrangeDetected;
  }

  @Override
  public boolean isHoldingFully() {
    return inputs.frontCANrangeDetected && inputs.backCANrangeDetected;
  }

  @Override
  public boolean isIdling() {
    return !inputs.frontCANrangeDetected && !inputs.backCANrangeDetected;
  }
}
