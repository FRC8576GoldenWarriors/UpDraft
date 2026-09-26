// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private boolean isHomed = false;

  public enum WantedState {
    IDLE,
    DEPLOY,
    RETRACT,
    INTAKE,
    HOME
  }

  private enum SystemState {
    IDLING,
    DEPLOYING,
    RETRACTING,
    INTAKING,
    HOMING
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(IntakeConstants.LOG_PATH, inputs);

    systemState = handleStateTransition();

    Logger.recordOutput(IntakeConstants.LOG_PATH + "WantedState", wantedState);
    Logger.recordOutput(IntakeConstants.LOG_PATH + "SystemState", wantedState);

    applyStates();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLING;
      case DEPLOY -> SystemState.DEPLOYING;
      case RETRACT -> SystemState.RETRACTING;
      case INTAKE -> SystemState.INTAKING;
      case HOME -> SystemState.HOMING;
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
      default -> idling();
    }
  }

  private void idling() {
    io.idle();
  }

  private void deploying() {
    io.setWantedIntakePosition(IntakeConstants.INTAKE_DOWN_POSITION);
    io.setWantedIntakeRollerSpeed(IntakeConstants.INTAKE_DEPLOYING_DUTY_CYCLE);
  }

  private void retracting() {
    io.setWantedIntakePosition(IntakeConstants.INTAKE_UP_POSITION);
    io.setWantedIntakeRollerSpeed(IntakeConstants.INTAKE_RETRACTING_DUTY_CYCLE);
  }

  private void intaking() {
    io.setWantedIntakePosition(IntakeConstants.INTAKE_DOWN_POSITION);
    io.setWantedIntakeRollerSpeed(IntakeConstants.INTAKE_INTAKING_DUTY_CYCLE);
  }

  private void homing() {
    this.isHomed = io.home(inputs.intakePivotCurrent);
    if (isHomed) {
      setWantedState(WantedState.IDLE);
    }
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
