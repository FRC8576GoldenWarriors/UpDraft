package frc.robot.subsystems.intake;

import frc.robot.util.LoggedSubsystem;

abstract class IntakeStateMachine extends LoggedSubsystem {

  public IntakeStateMachine(String logPath) {
    super(logPath);
  }

  protected abstract void idling();

  protected abstract void deploying();

  protected abstract void retracting();

  protected abstract void intaking();

  protected abstract void homing();

  protected abstract void positioningManually();
}
