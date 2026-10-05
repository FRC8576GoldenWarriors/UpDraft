package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.loggedimpl.LoggedSubsystem;

abstract class IntakeStateMachine extends LoggedSubsystem {

  public IntakeStateMachine(String logPath, StatusSignal<Current>[] intakeCurrentSignals) {
    super(logPath, intakeCurrentSignals);
  }

  protected abstract void idling();

  protected abstract void deploying();

  protected abstract void retracting();

  protected abstract void intaking();

  protected abstract void homing();

  protected abstract void positioningManually();
}
