package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.loggedimpl.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO extends LoggedIO {

  @AutoLog
  class ClawIOInputs {
    public boolean rollerConnected = false;
    public Angle rollerPosition = Rotations.zero();
    public AngularVelocity rollerVelocity = RotationsPerSecond.zero();
    public Voltage rollerVoltage = Volts.zero();
    public Current rollerStatorCurrent = Amps.zero();
    public Current rollerSupplyCurrent = Amps.zero();

    public boolean digitalInputCoral = false;
    public boolean digitalInputReefLeft = false;
    public boolean digitalInputReefRight = false;
  }

  void updateInputs(ClawIOInputs inputs);

  void idle();

  default void setWantedDutyCycle(double dutyCycle) {}
}
