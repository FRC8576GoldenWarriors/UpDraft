package frc.robot.subsystems.Transport;

import static edu.wpi.first.units.Units.Hertz;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TransportIOTalonFX {
    private final TalonFX transportMotorLeft;
    private final TalonFX transportMotorRight;
    private final TalonFXConfiguration transportMotorConfiguration;

      private final BooleanSupplier transportMotorLeftIsConnected;
        private final StatusSignal<AngularVelocity> transportAngularVelocityLeft;

    private final StatusSignal<Voltage> transportMotorLeftVoltage;
    private final StatusSignal<Current> transportMotorLeftSupplyCurrent;

    private final BooleanSupplier transportMotorRightIsConnected;
      private final StatusSignal<AngularVelocity> transportAngularVelocityRight;

    private final StatusSignal<Voltage> transportMotorRightVoltage;
    private final StatusSignal<Current> transportMotorRightSupplyCurrent;

    public TransportIOTalonFX() {
        transportMotorLeft = new TalonFX(transportConstants.TRANSPORT_LEFT_MOTOR_ID);
        transportMotorRight= new TalonFX(transportConstants.TRANSPORT_RIGHT_MOTOR_ID);
        transportMotorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(transportConstants.TRANSPORT_RIGHT_NEUTRAL_MODE_VALUE)
                    .withInverted(transportConstants.TRANSPORT_LEFT_INVERTED_VALUE)
            
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(transportConstants.TRANSPORT_SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(transportConstants.TRANSPORT_SUPPLY_CURRENT_LIMIT_ENABLED)
            );
        
        Slot0Configs slot0Configs = transportMotorConfiguration.Slot0;
        slot0Configs.kV = transportConstants.KV;
        slot0Configs.kP = transportConstants.KP;
        slot0Configs.kI = transportConstants.KI;
        slot0Configs.kD = transportConstants.KD;
        
        transportMotorLeftIsConnected = ()->transportMotorLeft.isConnected();
        transportMotorLeftVoltage = transportMotorLeft.getMotorVoltage();
        transportMotorLeftSupplyCurrent = transportMotorLeft.getSupplyCurrent();
        transportAngularVelocityLeft   =    transportMotorLeft.getVelocity();

        transportMotorRightIsConnected= ()->transportMotorRight.isConnected();
        transportMotorRightVoltage = transportMotorRight.getMotorVoltage();
        transportMotorRightSupplyCurrent =   transportMotorRight.getSupplyCurrent();
        transportAngularVelocityRight  =   transportMotorRight.getVelocity();

        transportMotorLeft.optimizeBusUtilization(Hertz.of(0));
        transportMotorRight.optimizeBusUtilization(Hertz.of(0));
        BaseStatusSignal.setUpdateFrequencyForAll(
            transportConstants.updateFrequency,
            statorCurrentStatusSignal,
            supplyCurrentStatusSignal,
            transportAngularVelocity,
            transportMotorVoltage,
            statorCurrentStatusSignalBack,
            supplyCurrentStatusSignalBack,
            transportAngularVelocityBack,
            transportMotorVoltageBack
        );
    }
}
