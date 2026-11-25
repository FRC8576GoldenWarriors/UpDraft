// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.util.loggedimpl.LoggedSystemState;
import frc.robot.util.loggedimpl.LoggedWantedState;
import frc.robot.util.loggedtunable.LoggedTunableDouble;

public class Elevator extends ElevatorStateMachine {

  public enum WantedState implements LoggedWantedState {
    IDLE,
    PICKUP_CORAL,
    CORAL_L1,
    CORAL_L2,
    CORAL_L3,
    CORAL_L4,
    ALAGE_L1,
    ALAGE_L2,
    ALAGE_FLOOR,
    ALAGE_LOLIPOP,
    BARGE,
    HOME,
    STANDBY,
    POSITION_MANUALLY,
    ZERO
  }

  private enum SystemState implements LoggedSystemState {
    IDLING,
    PICKING_UP_CORAL,
    MOVING_TO_CORAL_L1,
    MOVING_TO_CORAL_L2,
    MOVING_TO_CORAL_L3,
    MOVING_TO_CORAL_L4,
    MOVING_TO_ALAGE_L1,
    MOVING_TO_ALAGE_L2,
    MOVING_TO_ALAGE_FLOOR,
    MOVING_TO_ALAGE_LOLIPOP,
    MOVING_TO_BARGE,
    HOMING,
    STANDING_BY,
    POSITIONING_MANUALLY,
    ZEROING
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final LoggedTunableDouble manualPositionSetpoint;

  private boolean isHomed = false;

  public Elevator(ElevatorIO io) {
    super(ElevatorConstants.LOG_PATH, io.getCurrentSignals());
    this.io = io;
    manualPositionSetpoint =
        this.newSubsystemLoggedTunableDouble(
            "ManualSetpoint", (value) -> {}, ElevatorConstants.TUNING_MODE_ENABLED);
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
      case PICKUP_CORAL -> SystemState.PICKING_UP_CORAL;
      case CORAL_L1 -> SystemState.MOVING_TO_CORAL_L1;
      case CORAL_L2 -> SystemState.MOVING_TO_CORAL_L2;
      case CORAL_L3 -> SystemState.MOVING_TO_CORAL_L3;
      case CORAL_L4 -> SystemState.MOVING_TO_CORAL_L4;
      case ALAGE_L1 -> SystemState.MOVING_TO_ALAGE_L1;
      case ALAGE_L2 -> SystemState.MOVING_TO_ALAGE_L2;
      case ALAGE_FLOOR -> SystemState.MOVING_TO_ALAGE_FLOOR;
      case ALAGE_LOLIPOP -> SystemState.MOVING_TO_ALAGE_LOLIPOP;
      case BARGE -> SystemState.MOVING_TO_BARGE;
      case HOME -> SystemState.HOMING;
      case STANDBY -> SystemState.STANDING_BY;
      case POSITION_MANUALLY -> SystemState.POSITIONING_MANUALLY;
      case ZERO -> SystemState.ZEROING;
      default -> SystemState.IDLING;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLING -> idling();
      case PICKING_UP_CORAL -> pickingUpCoral();
      case MOVING_TO_CORAL_L1 -> movingToCoralL1();
      case MOVING_TO_CORAL_L2 -> movingToCoralL2();
      case MOVING_TO_CORAL_L3 -> movingToCoralL3();
      case MOVING_TO_CORAL_L4 -> movingToCoralL4();
      case MOVING_TO_ALAGE_L1 -> movingToAlageL1();
      case MOVING_TO_ALAGE_L2 -> movingToAlageL2();
      case MOVING_TO_ALAGE_FLOOR -> movingToAlageFloor();
      case MOVING_TO_ALAGE_LOLIPOP -> movingToAlageLolipop();
      case MOVING_TO_BARGE -> movingToBarge();
      case HOMING -> homing();
      case STANDING_BY -> standingBy();
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
  protected void pickingUpCoral() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'pickingUpCoral'");
  }

  @Override
  protected void movingToCoralL1() {
    io.setWantedPosition(Meters.of(0.5));
  }

  @Override
  protected void movingToCoralL2() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'movingToCoralL2'");
  }

  @Override
  protected void movingToCoralL3() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'movingToCoralL3'");
  }

  @Override
  protected void movingToCoralL4() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'movingToCoralL4'");
  }

  @Override
  protected void movingToAlageL1() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'movingToAlageL1'");
  }

  @Override
  protected void movingToAlageL2() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'movingToAlageL2'");
  }

  @Override
  protected void movingToAlageFloor() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'movingToAlageFloor'");
  }

  @Override
  protected void movingToAlageLolipop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'movingToAlageLolipop'");
  }

  @Override
  protected void movingToBarge() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'movingToBarge'");
  }

  @Override
  protected void homing() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'homing'");
  }

  @Override
  protected void standingBy() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'standingBy'");
  }

  @Override
  protected void positioningManually() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'positioningManually'");
  }

  @Override
  protected void zeroing() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'zeroing'");
  }
}
