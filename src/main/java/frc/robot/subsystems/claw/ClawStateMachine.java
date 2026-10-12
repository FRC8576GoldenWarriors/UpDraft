package frc.robot.subsystems.claw;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.loggedimpl.LoggedSubsystem;

abstract class ClawStateMachine extends LoggedSubsystem {

  public ClawStateMachine(String logPath, StatusSignal<Current>[] clawCurrentSignals) {
    super(logPath, clawCurrentSignals);
  }

  protected abstract void idling();

  protected abstract void scoringCoral();

  protected abstract void scoreingBargeAlage();

  protected abstract void scoreingProcessorAlage();

  protected abstract void pickingUpCoral();

  protected abstract void pickingUpFloorAlgae();

  protected abstract void pickingUpReefAlgae();

  protected abstract void dutyCycleManualing();
}
