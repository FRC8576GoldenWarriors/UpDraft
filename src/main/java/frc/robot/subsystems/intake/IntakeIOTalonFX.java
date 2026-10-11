package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.generated.TunerConstants;
import frc.robot.util.StatusSignalRefresher;
import frc.robot.util.alertmanager.AlertManager;
import frc.robot.util.loggedtunable.LoggedTunablePIDFConstants;
import java.util.HashSet;
import java.util.function.BooleanSupplier;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX pivotTalonFX;
  private final TalonFX rollerTalonFX;

  private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(Rotations.zero());
  private final DutyCycleOut pivotHomingRequest =
      new DutyCycleOut(IntakeConstants.PIVOT_HOMING_DUTY_CYCLE);
  private final DutyCycleOut rollerDutyCycleRequest = new DutyCycleOut(0);
  private final EmptyControl intakeIdleRequest = new EmptyControl();

  private final LoggedTunablePIDFConstants pivotLoggedTunablePIDFConstants;

  // Status Signals
  private final BooleanSupplier pivotConnectedSignal;
  private final StatusSignal<Angle> pivotPositionSignal;
  private final StatusSignal<AngularVelocity> pivotVelocitySignal;
  private final StatusSignal<Voltage> pivotVoltageSignal;
  private final StatusSignal<Current> pivotStatorCurrentSignal;
  private final StatusSignal<Current> pivotSupplyCurrentSignal;

  private final BooleanSupplier rollerConnectedSignal;
  private final StatusSignal<AngularVelocity> rollerVelocitySignal;
  private final StatusSignal<Voltage> rollerVoltageSignal;
  private final StatusSignal<Current> rollerStatorCurrentSignal;
  private final StatusSignal<Current> rollerSupplyCurrentSignal;

  private final HashSet<StatusSignal<Current>> intakeCurrentSignals = new HashSet<>();

  public IntakeIOTalonFX() {
    pivotTalonFX = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, TunerConstants.kCANBus);
    rollerTalonFX = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, TunerConstants.kCANBus);

    pivotLoggedTunablePIDFConstants =
        new LoggedTunablePIDFConstants(
            IntakeConstants.PIVOT_TUNING_PATH,
            IntakeConstants.TUNING_MODE_ENABLED,
            (values) -> {
              pivotTalonFX
                  .getConfigurator()
                  .apply(
                      IntakeConstants.PIVOT_CONFIG
                          .Slot0
                          .withKP(values[0])
                          .withKI(values[1])
                          .withKD(values[2])
                          .withKS(values[3])
                          .withKG(values[4])
                          .withKV(values[5])
                          .withKA(values[6]));
            },
            new double[] {
              IntakeConstants.KP,
              IntakeConstants.KI,
              IntakeConstants.KD,
              IntakeConstants.KS,
              IntakeConstants.KG,
              IntakeConstants.KV,
              IntakeConstants.KA
            });

    pivotConnectedSignal = () -> pivotTalonFX.isConnected();
    pivotPositionSignal = pivotTalonFX.getPosition();
    pivotVelocitySignal = pivotTalonFX.getVelocity();
    pivotVoltageSignal = pivotTalonFX.getMotorVoltage();
    pivotStatorCurrentSignal = pivotTalonFX.getStatorCurrent();
    pivotSupplyCurrentSignal = pivotTalonFX.getSupplyCurrent();
    intakeCurrentSignals.add(pivotSupplyCurrentSignal);

    AlertManager.getInstance()
        .registerAlert(
            new Alert("The Intake Pivot Motor is Disconnected.", AlertType.kError),
            pivotConnectedSignal);

    rollerConnectedSignal = () -> rollerTalonFX.isConnected();
    rollerVelocitySignal = rollerTalonFX.getVelocity();
    rollerVoltageSignal = rollerTalonFX.getMotorVoltage();
    rollerStatorCurrentSignal = rollerTalonFX.getStatorCurrent();
    rollerSupplyCurrentSignal = rollerTalonFX.getSupplyCurrent();
    intakeCurrentSignals.add(rollerSupplyCurrentSignal);

    AlertManager.getInstance()
        .registerAlert(
            new Alert("The Intake Roller Motor is Disconnected.", AlertType.kError),
            rollerConnectedSignal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
        pivotPositionSignal,
        pivotVelocitySignal,
        pivotVoltageSignal,
        pivotStatorCurrentSignal,
        pivotSupplyCurrentSignal,
        rollerVelocitySignal,
        rollerVoltageSignal,
        rollerStatorCurrentSignal,
        rollerSupplyCurrentSignal);

    StatusSignalRefresher.getInstance()
        .addStatusSignals(
            pivotPositionSignal,
            pivotVelocitySignal,
            pivotVoltageSignal,
            pivotStatorCurrentSignal,
            pivotSupplyCurrentSignal,
            rollerVelocitySignal,
            rollerVoltageSignal,
            rollerStatorCurrentSignal,
            rollerSupplyCurrentSignal);

    pivotTalonFX.getConfigurator().apply(IntakeConstants.PIVOT_CONFIG);
    rollerTalonFX.getConfigurator().apply(IntakeConstants.ROLLER_CONFIG);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotConnected = pivotConnectedSignal.getAsBoolean();
    inputs.pivotPosition = pivotPositionSignal.getValue();
    inputs.pivotVelocity = pivotVelocitySignal.getValue();
    inputs.pivotVoltage = pivotVoltageSignal.getValue();
    inputs.pivotStatorCurrent = pivotStatorCurrentSignal.getValue();
    inputs.pivotSupplyCurrent = pivotSupplyCurrentSignal.getValue();

    inputs.rollerConnected = rollerConnectedSignal.getAsBoolean();
    inputs.rollerVelocity = rollerVelocitySignal.getValue();
    inputs.rollerVoltage = rollerVoltageSignal.getValue();
    inputs.rollerStatorCurrent = rollerStatorCurrentSignal.getValue();
    inputs.rollerSupplyCurrent = rollerSupplyCurrentSignal.getValue();
  }

  @Override
  public void setWantedPosition(Angle position) {
    pivotTalonFX.setControl(pivotPositionRequest.withPosition(position));
  }

  @Override
  public void setWantedRollerSpeed(double dutyCycleOutput) {
    rollerTalonFX.setControl(rollerDutyCycleRequest.withOutput(dutyCycleOutput));
  }

  @Override
  public void idle() {
    pivotTalonFX.setControl(intakeIdleRequest);
    rollerTalonFX.setControl(intakeIdleRequest);
  }

  @Override
  public boolean home(Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    if (!atUpperHomingHardstop(activeHomingCurrent, activeHomingVelocity)) {
      pivotTalonFX.setControl(pivotHomingRequest);
      return false;
    }
    pivotTalonFX.setPosition(IntakeConstants.EXPECTED_HOMING_ZERO);
    return true;
  }

  @Override
  public boolean nearSetpoint() {
    return pivotPositionSignal.isNear(
        pivotPositionRequest.getPositionMeasure(), IntakeConstants.PIVOT_POSITION_TOLERANCE);
  }

  @Override
  public StatusSignal<Current>[] getCurrentSignals() {
    return intakeCurrentSignals.toArray(StatusSignal[]::new);
  }

  private boolean atUpperHomingHardstop(
      Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    return pivotStatorCurrentSignal.isNear(
            IntakeConstants.PIVOT_HOMING_CURRENT, IntakeConstants.PIVOT_HOMING_CURRENT_TOLERANCE)
        && pivotVelocitySignal.isNear(
            IntakeConstants.PIVOT_HOMING_VELOCITY, IntakeConstants.PIVOT_HOMING_VELOCITY_TOLERANCE);
  }
}
