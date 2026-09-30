package frc.robot.subsystems.Transport;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TransportIO {

    
    @AutoLog
    public class TransportIOInputs{

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

    default void setTransportVoltage(double volts) {}


    default void setTransportSpeed(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {}
    
}
