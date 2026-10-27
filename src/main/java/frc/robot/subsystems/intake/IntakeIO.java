package frc.robot.subsystems.intake;

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

public interface IntakeIO extends LoggedIO {

  void updateInputs(IntakeIOInputs inputs);

  void idle();

  @AutoLog
  class IntakeIOInputs {
    public boolean pivotConnected = false;
    public Angle pivotPosition = Rotations.zero();
    public AngularVelocity pivotVelocity = RotationsPerSecond.zero();
    public Voltage pivotVoltage = Volts.zero();
    public Current pivotStatorCurrent = Amps.zero();
    public Current pivotSupplyCurrent = Amps.zero();

    public boolean rollerConnected = false;
    public AngularVelocity rollerVelocity = RotationsPerSecond.zero();
    public Voltage rollerVoltage = Volts.zero();
    public Current rollerStatorCurrent = Amps.zero();
    public Current rollerSupplyCurrent = Amps.zero();
  }

  default void setWantedPosition(Angle position) {}

  default void setWantedRollerSpeed(double dutyCycleOutput) {}

  default void zero() {}

  default boolean home() {
    return false;
  }

  default boolean nearSetpoint() {
    return false;
  }
}
