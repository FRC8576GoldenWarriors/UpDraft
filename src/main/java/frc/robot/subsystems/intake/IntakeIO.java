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
    public boolean intakePivotConnected = false;
    public Angle intakePivotPosition = Rotations.zero();
    public AngularVelocity intakePivotVelocity = RotationsPerSecond.zero();
    public Voltage intakePivotVoltage = Volts.zero();
    public Current intakePivotCurrent = Amps.zero();

    public boolean intakeRollerConnected = false;
    public AngularVelocity intakeRollerVelocity = RotationsPerSecond.zero();
    public Voltage intakeRollerVoltage = Volts.zero();
    public Current intakeRollerCurrent = Amps.zero();
  }

  default void setWantedIntakePosition(Angle position) {}

  default void setWantedIntakeRollerSpeed(double dutyCycleOutput) {}

  default boolean home(Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    return false;
  }

  default boolean nearSetpoint() {
    return false;
  }
}
