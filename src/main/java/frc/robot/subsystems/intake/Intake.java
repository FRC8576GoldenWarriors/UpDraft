// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import frc.robot.util.loggedtunable.LoggedTunableDouble;
import org.littletonrobotics.junction.Logger;

public class Intake extends IntakeStateMachine {

  private IntakeIO io;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final LoggedTunableDouble manualPositionSetpoint;

  private boolean isHomed = false;

  public enum WantedState {
    IDLE,
    DEPLOY,
    RETRACT,
    INTAKE,
    HOME,
    POSITION_MANUALLY
  }

  private enum SystemState {
    IDLING,
    DEPLOYING,
    RETRACTING,
    INTAKING,
    HOMING,
    POSITIONING_MANUALLY
  }

  public Intake(IntakeIO io) {
    this.io = io;
    manualPositionSetpoint =
        new LoggedTunableDouble(
            IntakeConstants.LOG_PATH + "ManualSetpoint",
            (value) -> {},
            IntakeConstants.INTAKE_TUNING_MODE_ENABLED);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(IntakeConstants.LOG_PATH, inputs);

    systemState = handleStateTransition();

    Logger.recordOutput(IntakeConstants.LOG_PATH + "WantedState", wantedState);
    Logger.recordOutput(IntakeConstants.LOG_PATH + "SystemState", wantedState);

    Logger.recordOutput(IntakeConstants.LOG_PATH + "IsHomed", isHomed);

    applyStates();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLING;
      case DEPLOY -> SystemState.DEPLOYING;
      case RETRACT -> SystemState.RETRACTING;
      case INTAKE -> SystemState.INTAKING;
      case HOME -> SystemState.HOMING;
      case POSITION_MANUALLY -> SystemState.POSITIONING_MANUALLY;
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
      default -> idling();
    }
  }

  @Override
  protected void idling() {
    io.idle();
  }

  @Override
  protected void deploying() {
    io.setWantedIntakePosition(IntakeConstants.INTAKE_DOWN_POSITION);
    io.setWantedIntakeRollerSpeed(IntakeConstants.INTAKE_DEPLOYING_DUTY_CYCLE);
  }

  @Override
  protected void retracting() {
    io.setWantedIntakePosition(IntakeConstants.INTAKE_UP_POSITION);
    io.setWantedIntakeRollerSpeed(IntakeConstants.INTAKE_RETRACTING_DUTY_CYCLE);
  }

  @Override
  protected void intaking() {
    io.setWantedIntakePosition(IntakeConstants.INTAKE_DOWN_POSITION);
    io.setWantedIntakeRollerSpeed(IntakeConstants.INTAKE_INTAKING_DUTY_CYCLE);
  }

  @Override
  protected void positioningManually() {
    io.setWantedIntakePosition(Rotations.of(manualPositionSetpoint.get()));
    io.setWantedIntakeRollerSpeed(0);
  }

  @Override
  protected void homing() {
    this.isHomed = io.home(inputs.intakePivotCurrent, inputs.intakePivotVelocity);
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
