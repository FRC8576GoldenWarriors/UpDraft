package frc.robot.subsystems.Transport;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.loggedimpl.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface TransportIO extends LoggedIO {

  @AutoLog
  public class TransportIOInputs {

    Voltage transportMotorLeftVoltage = Volts.of(0);
    Voltage transportMotorRightVoltage = Volts.of(0);

    Current transportMotorRightStatorCurrent = Amps.of(0);
    Current transportMotorLeftStatorCurrent = Amps.of(0);

    Current transportMotorRightSupplyCurrent = Amps.of(0);
    Current transportMotorLeftSupplyCurrent = Amps.of(0);

    AngularVelocity transportAngularVelocityRight = RotationsPerSecond.of(0.0);
    AngularVelocity transportAngularVelocityLeft = RotationsPerSecond.of(0.0);

    boolean transportMotorLeftIsConnected = false;
    boolean transportMotorRightIsConnected = false;
  }

  default void updateInputs(TransportIOInputs inputs) {}

  default void setTransportDutyCycle(double output) {}

  default void setTransportDutyCycle(double leftOutput, double rightOutput) {}

  default void idle() {}

  default void setTransportSpeed(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {}
}
