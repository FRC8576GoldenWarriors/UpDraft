package frc.robot.subsystems.Transport;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Transport.TransportIO.TransportIOInputs;
import frc.robot.util.StatusSignalRefresher;
import java.util.HashSet;
import java.util.function.BooleanSupplier;

public class TransportIOTalonFx implements TransportIO {
  private final TalonFX transportMotorLeft;
  private final TalonFX transportMotorRight;
  private final TalonFXConfiguration transportMotorConfiguration;

  private final BooleanSupplier transportMotorLeftIsConnected;
  private final StatusSignal<AngularVelocity> transportAngularVelocityLeft;

  private final StatusSignal<Voltage> transportMotorLeftVoltage;
  private final StatusSignal<Current> transportMotorLeftSupplyCurrent;
  private final StatusSignal<Current> transportMotorLeftStatorCurrent;

  private final BooleanSupplier transportMotorRightIsConnected;
  private final StatusSignal<AngularVelocity> transportAngularVelocityRight;

  private final StatusSignal<Voltage> transportMotorRightVoltage;
  private final StatusSignal<Current> transportMotorRightSupplyCurrent;
  private final StatusSignal<Current> transportMotorRightStatorCurrent;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final DutyCycleOut dutyOut = new DutyCycleOut(0);
  private final EmptyControl idleRequest = new EmptyControl();

  private final HashSet<StatusSignal<Current>> transportCurrentSignals = new HashSet<>();

  public TransportIOTalonFx() {
    transportMotorLeft = new TalonFX(transportConstants.TRANSPORT_LEFT_MOTOR_ID);
    transportMotorRight = new TalonFX(transportConstants.TRANSPORT_RIGHT_MOTOR_ID);
    transportMotorConfiguration =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(transportConstants.TRANSPORT_RIGHT_NEUTRAL_MODE_VALUE)
                    .withInverted(transportConstants.TRANSPORT_LEFT_INVERTED_VALUE))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(transportConstants.TRANSPORT_SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(
                        transportConstants.TRANSPORT_SUPPLY_CURRENT_LIMIT_ENABLED));

    Slot0Configs slot0Configs = transportMotorConfiguration.Slot0;
    slot0Configs.kV = transportConstants.KV;
    slot0Configs.kP = transportConstants.KP;
    slot0Configs.kI = transportConstants.KI;
    slot0Configs.kD = transportConstants.KD;

    transportMotorLeftIsConnected = () -> transportMotorLeft.isConnected();
    transportMotorLeftVoltage = transportMotorLeft.getMotorVoltage();
    transportMotorLeftSupplyCurrent = transportMotorLeft.getSupplyCurrent();
    transportMotorLeftStatorCurrent = transportMotorLeft.getStatorCurrent();
    transportAngularVelocityLeft = transportMotorLeft.getVelocity();

    transportMotorRightIsConnected = () -> transportMotorRight.isConnected();
    transportMotorRightVoltage = transportMotorRight.getMotorVoltage();
    transportMotorRightSupplyCurrent = transportMotorRight.getSupplyCurrent();
    transportMotorRightStatorCurrent = transportMotorRight.getStatorCurrent();
    transportAngularVelocityRight = transportMotorRight.getVelocity();

    transportCurrentSignals.add(transportMotorRightStatorCurrent);
    transportCurrentSignals.add(transportMotorLeftStatorCurrent);

    transportMotorLeft.optimizeBusUtilization(Hertz.of(0));
    transportMotorRight.optimizeBusUtilization(Hertz.of(0));
    BaseStatusSignal.setUpdateFrequencyForAll(
        transportConstants.updateFrequency,
        transportMotorLeftVoltage,
        transportMotorLeftSupplyCurrent,
        transportAngularVelocityLeft,
        transportMotorRightVoltage,
        transportMotorRightSupplyCurrent,
        transportAngularVelocityRight);

    StatusSignalRefresher.getInstance()
        .addStatusSignals(
            transportMotorLeftVoltage,
            transportMotorLeftSupplyCurrent,
            transportAngularVelocityLeft,
            transportMotorRightVoltage,
            transportMotorRightSupplyCurrent,
            transportAngularVelocityRight);

    transportMotorLeft.getConfigurator().apply(transportMotorConfiguration);
    transportMotorConfiguration.MotorOutput.Inverted =
        transportConstants.TRANSPORT_RIGHT_INVERTED_VALUE;
    transportMotorRight.getConfigurator().apply(transportMotorConfiguration);
  }

  @Override
  public void updateInputs(TransportIOInputs inputs) {

    inputs.transportMotorLeftVoltage = transportMotorLeftVoltage.getValue();
    inputs.transportMotorRightVoltage = transportMotorRightVoltage.getValue();
    inputs.transportMotorRightStatorCurrent = transportMotorRightStatorCurrent.getValue();
    inputs.transportMotorLeftStatorCurrent = transportMotorLeftStatorCurrent.getValue();
    inputs.transportMotorRightSupplyCurrent = transportMotorRightSupplyCurrent.getValue();
    inputs.transportMotorLeftSupplyCurrent = transportMotorLeftSupplyCurrent.getValue();
    inputs.transportAngularVelocityRight = transportAngularVelocityRight.getValue();
    inputs.transportAngularVelocityLeft = transportAngularVelocityLeft.getValue();
    inputs.transportAngularVelocityLeft = transportAngularVelocityLeft.getValue();

    inputs.transportMotorLeftIsConnected = transportMotorLeftIsConnected.getAsBoolean();
    inputs.transportMotorRightIsConnected = transportMotorRightIsConnected.getAsBoolean();
  }

  @Override
  public void setTransportDutyCycle(double output) {
    transportMotorLeft.setControl(dutyOut.withOutput(output));
    transportMotorRight.setControl(dutyOut.withOutput(output));
  }

  @Override
  public void setTransportDutyCycle(double leftOutput, double rightOutput) {
    transportMotorLeft.setControl(dutyOut.withOutput(leftOutput));
    transportMotorRight.setControl(dutyOut.withOutput(rightOutput));
  }

  @Override
  public void idle() {
    transportMotorLeft.setControl(idleRequest);
    transportMotorRight.setControl(idleRequest);
  }

  @Override
  public void setTransportSpeed(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {
    transportMotorLeft.setControl(velocityRequest.withVelocity(leftVelocity));
    transportMotorRight.setControl(velocityRequest.withVelocity(rightVelocity));
  }

  @Override
  public StatusSignal<Current>[] getCurrentSignals() {
    return transportCurrentSignals.toArray(StatusSignal[]::new);
  }
}
