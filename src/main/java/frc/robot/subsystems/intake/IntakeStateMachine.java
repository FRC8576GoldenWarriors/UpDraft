package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

abstract class IntakeStateMachine extends SubsystemBase {
  protected abstract void idling();

  protected abstract void deploying();

  protected abstract void retracting();

  protected abstract void intaking();

  protected abstract void homing();

  protected abstract void positioningManually();
}
