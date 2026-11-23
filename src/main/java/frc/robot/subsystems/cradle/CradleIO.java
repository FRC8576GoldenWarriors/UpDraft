package frc.robot.subsystems.cradle;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.util.loggedimpl.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface CradleIO extends LoggedIO {
  void updateInputs(CradleIOInputs inputs);

  @AutoLog
  class CradleIOInputs {
    public boolean frontCANrangeConnected = false;
    public boolean frontCANrangeDetected = false;
    public Distance frontCANrangeDistance = Meters.zero();

    public boolean backCANrangeConnected = false;
    public boolean backCANrangeDetected = false;
    public Distance backCANrangeDistance = Meters.zero();
  }
}
