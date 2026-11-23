package frc.robot.subsystems.cradle;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.loggedimpl.LoggedSubsystem;

abstract class CradleStateMachine extends LoggedSubsystem {

  public CradleStateMachine(String logPath, StatusSignal<Current>[] cradleCurrentSignals) {
    super(logPath, cradleCurrentSignals);
  }

  public abstract boolean isHoldingPartially();

  public abstract boolean isHoldingFully();

  public abstract boolean isIdling();
}
