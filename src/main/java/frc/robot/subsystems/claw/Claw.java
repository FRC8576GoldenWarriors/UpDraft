package frc.robot.subsystems.claw;

import frc.robot.util.loggedimpl.LoggedSystemState;
import frc.robot.util.loggedimpl.LoggedWantedState;
import frc.robot.util.loggedtunable.LoggedTunableDouble;

public class Claw extends ClawStateMachine {

  public enum WantedState implements LoggedWantedState {
    IDLE,
    SCORE_CORAL,
    SCORE_BARGE_ALAGE,
    SCORE_PROCESSOR_ALAGE,
    PICKUP_CORAL,
    PICKUP_FLOOR_ALAGE,
    PICKUP_REEF_ALAGE,
    DUTY_CYCLE_MANUAL
  }

  private enum SystemState implements LoggedSystemState {
    IDLING,
    SCORING_CORAL,
    SCOREING_BARGE_ALAGE,
    SCOREING_PROCESSOR_ALAGE,
    PICKING_UP_CORAL,
    PICKING_UP_FLOOR_ALGAE,
    PICKING_UP_REEF_ALGAE,
    DUTY_CYCLE_MANUALING
  }

  private final ClawIO io;

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final LoggedTunableDouble manualDutyCycleSetpoint;

  public Claw(ClawIO io) {
    super(ClawConstants.LOG_PATH, io.getCurrentSignals());
    this.io = io;
    manualDutyCycleSetpoint =
        newSubsystemLoggedTunableDouble(
            ClawConstants.TUNING_PATH, (value) -> {}, ClawConstants.TUNING_MODE_ENABLED);
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();

    recordWantedState(wantedState);
    recordSystemState(systemState);

    applyStates();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLING;
      case SCORE_CORAL -> SystemState.SCORING_CORAL;
      case SCORE_BARGE_ALAGE -> SystemState.SCOREING_BARGE_ALAGE;
      case SCORE_PROCESSOR_ALAGE -> SystemState.SCOREING_PROCESSOR_ALAGE;
      case PICKUP_CORAL -> SystemState.PICKING_UP_CORAL;
      case PICKUP_FLOOR_ALAGE -> SystemState.PICKING_UP_FLOOR_ALGAE;
      case PICKUP_REEF_ALAGE -> SystemState.PICKING_UP_REEF_ALGAE;
      case DUTY_CYCLE_MANUAL -> SystemState.DUTY_CYCLE_MANUALING;
      default -> SystemState.IDLING;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLING -> idling();
      case SCORING_CORAL -> scoringCoral();
      case SCOREING_BARGE_ALAGE -> scoreingBargeAlage();
      case SCOREING_PROCESSOR_ALAGE -> scoreingProcessorAlage();
      case PICKING_UP_CORAL -> pickingUpCoral();
      case PICKING_UP_FLOOR_ALGAE -> pickingUpFloorAlgae();
      case PICKING_UP_REEF_ALGAE -> pickingUpReefAlgae();
      case DUTY_CYCLE_MANUALING -> dutyCycleManualing();
      default -> idling();
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  @Override
  protected void idling() {
    io.idle();
  }

  @Override
  protected void scoringCoral() {
    io.setWantedDutyCycle(ClawConstants.SCORE_CORAL_DUTY_CYCLE);
  }

  @Override
  protected void scoreingBargeAlage() {
    io.setWantedDutyCycle(ClawConstants.SCORE_BARGE_ALAGE_DUTY_CYCLE);
  }

  @Override
  protected void scoreingProcessorAlage() {
    io.setWantedDutyCycle(ClawConstants.SCORE_PROCESSOR_ALAGE_DUTY_CYCLE);
  }

  @Override
  protected void pickingUpCoral() {
    io.setWantedDutyCycle(ClawConstants.PICKUP_CORAL_DUTY_CYCLE);
  }

  @Override
  protected void pickingUpFloorAlgae() {
    io.setWantedDutyCycle(ClawConstants.PICKUP_FLOOR_ALAGE_DUTY_CYCLE);
  }

  @Override
  protected void pickingUpReefAlgae() {
    io.setWantedDutyCycle(ClawConstants.PICKUP_REEF_ALAGE_DUTY_CYCLE);
  }

  @Override
  protected void dutyCycleManualing() {
    io.setWantedDutyCycle(manualDutyCycleSetpoint.get());
  }
}
