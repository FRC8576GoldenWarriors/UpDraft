package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  void updateInputs(IntakeIOInputs inputs);

  @AutoLog
  class IntakeIOInputs {
    public boolean intakePivotConnected = false;
    public Angle intakePivotPosition = Radians.zero();
    public AngularVelocity intakePivotVelocity = RadiansPerSecond.zero();
    public Voltage intakePivotVoltage = Volts.zero();
    public Current intakePivotCurrent = Amps.zero();

    public boolean intakeRollerConnected = false;
    public AngularVelocity intakeRollerVelocity = RadiansPerSecond.zero();
    public Voltage intakeRollerVoltage = Volts.zero();
    public Current intakeRollerCurrent = Amps.zero();
  }

  default void setWantedIntakePosition(Angle position) {}

  default void setWantedIntakeRollerSpeed(double dutyCycleOutput) {}

  default void idle() {}

  default boolean home(Current activeHomingCurrent) {
    return false;
  }
}
