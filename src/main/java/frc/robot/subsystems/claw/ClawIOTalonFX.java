package frc.robot.subsystems.claw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.generated.TunerConstants;
import frc.robot.util.StatusSignalRefresher;
import frc.robot.util.alertmanager.AlertManager;
import java.util.HashSet;
import java.util.function.BooleanSupplier;

public class ClawIOTalonFX implements ClawIO {

  private final TalonFX rollerTalonFX;

  private final DigitalInput coralPhotoelectric;
  private final DigitalInput leftReefPhotoelectric;
  private final DigitalInput rightReefPhotoelectric;

  private final DutyCycleOut rollerDutyCycleRequest = new DutyCycleOut(0);
  private final EmptyControl rollerIdleRequest = new EmptyControl();

  private final BooleanSupplier rollerConnectedSignal;
  private final StatusSignal<Angle> rollerPositionSignal;
  private final StatusSignal<AngularVelocity> rollerVelocitySignal;
  private final StatusSignal<Voltage> rollerVoltageSignal;
  private final StatusSignal<Current> rollerStatorCurrentSignal;
  private final StatusSignal<Current> rollerSupplyCurrentSignal;

  private final HashSet<StatusSignal<Current>> rollerCurrentSignals = new HashSet<>();

  public ClawIOTalonFX() {
    rollerTalonFX = new TalonFX(ClawConstants.ROLLER_MOTOR_ID, TunerConstants.kCANBus);

    coralPhotoelectric = new DigitalInput(ClawConstants.CORAL_DIGITAL_INPUT_ID);
    leftReefPhotoelectric = new DigitalInput(ClawConstants.LEFT_REEF_DIGITAL_INPUT_ID);
    rightReefPhotoelectric = new DigitalInput(ClawConstants.RIGHT_REEF_DIGITAL_INPUT_ID);

    rollerConnectedSignal = () -> rollerTalonFX.isConnected();
    rollerPositionSignal = rollerTalonFX.getPosition();
    rollerVelocitySignal = rollerTalonFX.getVelocity();
    rollerVoltageSignal = rollerTalonFX.getMotorVoltage();
    rollerStatorCurrentSignal = rollerTalonFX.getStatorCurrent();
    rollerSupplyCurrentSignal = rollerTalonFX.getSupplyCurrent();
    rollerCurrentSignals.add(rollerSupplyCurrentSignal);

    AlertManager.getInstance()
        .registerAlert(
            new Alert("The Claw Roller Motor is Disconnected", AlertType.kError),
            rollerConnectedSignal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClawConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
        rollerPositionSignal,
        rollerVelocitySignal,
        rollerVoltageSignal,
        rollerStatorCurrentSignal,
        rollerSupplyCurrentSignal);

    StatusSignalRefresher.getInstance()
        .addStatusSignals(
            rollerPositionSignal,
            rollerVelocitySignal,
            rollerVoltageSignal,
            rollerStatorCurrentSignal,
            rollerSupplyCurrentSignal);

    rollerTalonFX.getConfigurator().apply(ClawConstants.ROLLER_CONFIG);
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.rollerConnected = rollerConnectedSignal.getAsBoolean();
    inputs.rollerPosition = rollerPositionSignal.getValue();
    inputs.rollerVelocity = rollerVelocitySignal.getValue();
    inputs.rollerVoltage = rollerVoltageSignal.getValue();
    inputs.rollerStatorCurrent = rollerStatorCurrentSignal.getValue();
    inputs.rollerSupplyCurrent = rollerSupplyCurrentSignal.getValue();

    inputs.digitalInputCoral = coralPhotoelectric.get();
    inputs.digitalInputReefLeft = leftReefPhotoelectric.get();
    inputs.digitalInputReefRight = rightReefPhotoelectric.get();
  }

  @Override
  public StatusSignal<Current>[] getCurrentSignals() {
    return rollerCurrentSignals.toArray(StatusSignal[]::new);
  }

  @Override
  public void idle() {
    rollerTalonFX.setControl(rollerIdleRequest);
  }

  @Override
  public void setWantedDutyCycle(double dutyCycle) {
    rollerTalonFX.setControl(rollerDutyCycleRequest.withOutput(dutyCycle));
  }
}
