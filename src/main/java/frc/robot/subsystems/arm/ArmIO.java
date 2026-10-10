package frc.robot.subsystems.arm;

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

public interface ArmIO extends LoggedIO {

  void updateInputs(ArmIOInputs inputs);

  @AutoLog
  class ArmIOInputs {
    public boolean armConnected = false;
    public Angle armPosition = Rotations.zero();
    public AngularVelocity armVelocity = RotationsPerSecond.zero();
    public Voltage armVoltage = Volts.zero();
    public Current armCurrent = Amps.zero();
  }

  void idle();

  default void setWantedArmPosition(Angle position) {}

  default boolean home(Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    return false;
  }

  default boolean nearSetpoint() {
    return false;
  }
}
