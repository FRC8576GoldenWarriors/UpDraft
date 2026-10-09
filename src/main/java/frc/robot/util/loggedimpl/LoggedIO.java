package frc.robot.util.loggedimpl;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;

public interface LoggedIO {

  StatusSignal<Current>[] getCurrentSignals();
}
