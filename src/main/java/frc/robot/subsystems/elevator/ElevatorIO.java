package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.loggedimpl.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO extends LoggedIO {
  void updateInputs(ElevatorIOInputs inputs);

  void idle();

  @AutoLog
  class ElevatorIOInputs {
    public boolean leftMotorConnected = false;
    public Voltage leftMotorVoltage = Volts.zero();
    public Current leftMotorStatorCurrent = Amps.zero();
    public Current leftMotorSupplyCurrent = Amps.zero();

    public boolean rightMotorConnected = false;
    public Voltage rightMotorVoltage = Volts.zero();
    public Current rightMotorStatorCurrent = Amps.zero();
    public Current rightMotorSupplyCurrent = Amps.zero();

    public Angle rotationalPosition = Rotations.zero();
    public AngularVelocity rotationalVelocity = RotationsPerSecond.zero();

    public Distance linearPosition = Meters.zero();
    public LinearVelocity linearVelocity = MetersPerSecond.zero();
  }

  void setWantedPosition(Angle rotationalPosition);

  void setWantedPosition(Distance linearPosition);

  boolean home();

  boolean nearSetpoint();

  void zero();
}
