package frc.robot.subsystems.Transport;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.loggedimpl.LoggedSubsystem;

abstract class TransportStateMachine extends LoggedSubsystem {

  public TransportStateMachine(String logPath, StatusSignal<Current>[] transportCurrentSignals) {
    super(logPath, transportCurrentSignals);
  }

  protected abstract void idling();

  protected abstract void transportingIn();

  protected abstract void transportingOut();
}
