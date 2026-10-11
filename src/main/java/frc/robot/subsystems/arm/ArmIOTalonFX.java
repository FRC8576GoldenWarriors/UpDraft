package frc.robot.subsystems.arm;

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
import frc.robot.util.PIDFConstants;
import frc.robot.util.StatusSignalRefresher;
import frc.robot.util.alertmanager.AlertManager;
import frc.robot.util.loggedtunable.LoggedTunablePIDFConstants;
import java.util.HashSet;
import java.util.function.BooleanSupplier;

public class ArmIOTalonFX implements ArmIO {

  private final TalonFX pivotTalonFX;

  private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(Rotations.zero());
  private final DutyCycleOut pivotHomingRequest =
      new DutyCycleOut(ArmConstants.PIVOT_HOMING_DUTY_CYCLE);
  private final EmptyControl pivotIdleRequest = new EmptyControl();

  private final LoggedTunablePIDFConstants pivotLoggedTunablePIDFConstants;

  private final BooleanSupplier pivotConnectedSignal;
  private final StatusSignal<Angle> pivotPositionSignal;
  private final StatusSignal<AngularVelocity> pivotVelocitySignal;
  private final StatusSignal<Voltage> pivotVoltageSignal;
  private final StatusSignal<Current> pivotCurrentSignal;

  private final HashSet<StatusSignal<Current>> pivotCurrentSignals = new HashSet<>();

  public ArmIOTalonFX() {
    pivotTalonFX = new TalonFX(ArmConstants.PIVOT_MOTOR, TunerConstants.kCANBus);

    pivotLoggedTunablePIDFConstants =
        new LoggedTunablePIDFConstants(
            ArmConstants.PIVOT_TUNING_PATH,
            ArmConstants.TUNING_MODE_ENABLED,
            (values) -> {
              pivotTalonFX
                  .getConfigurator()
                  .apply(
                      ArmConstants.PIVOT_CONFIG
                          .Slot0
                          .withKP(values[0])
                          .withKI(values[1])
                          .withKD(values[2])
                          .withKS(values[3])
                          .withKG(values[4])
                          .withKV(values[5])
                          .withKA(values[6]));
            },
            PIDFConstants.toDoubleArray(ArmConstants.PIDF_CONSTANTS));

    pivotConnectedSignal = () -> pivotTalonFX.isConnected();
    pivotPositionSignal = pivotTalonFX.getPosition();
    pivotVelocitySignal = pivotTalonFX.getVelocity();
    pivotVoltageSignal = pivotTalonFX.getMotorVoltage();
    pivotCurrentSignal = pivotTalonFX.getStatorCurrent();
    pivotCurrentSignals.add(pivotTalonFX.getSupplyCurrent());

    AlertManager.getInstance()
        .registerAlert(
            new Alert("The Arm Pivot Motor Disconnected.", AlertType.kError), pivotConnectedSignal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ArmConstants.PIVOT_STATUS_SIGNAL_UPDATE_FREQUENCY,
        pivotPositionSignal,
        pivotVelocitySignal,
        pivotVoltageSignal,
        pivotCurrentSignal);

    StatusSignalRefresher.getInstance()
        .addStatusSignals(
            pivotPositionSignal, pivotVelocitySignal, pivotVoltageSignal, pivotCurrentSignal);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.pivotConnected = pivotConnectedSignal.getAsBoolean();
    inputs.pivotPosition = pivotPositionSignal.getValue();
    inputs.pivotVelocity = pivotVelocitySignal.getValue();
    inputs.pivotVoltage = pivotVoltageSignal.getValue();
    inputs.pivotCurrent = pivotCurrentSignal.getValue();
  }

  @Override
  public StatusSignal<Current>[] getCurrentSignals() {
    return pivotCurrentSignals.toArray(StatusSignal[]::new);
  }

  @Override
  public void idle() {
    pivotTalonFX.setControl(pivotIdleRequest);
  }

  @Override
  public void setWantedArmPosition(Angle position) {
    pivotTalonFX.setControl(pivotPositionRequest.withPosition(position));
  }

  @Override
  public boolean home(Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    if (!atUpperHomingHardstop(activeHomingCurrent, activeHomingVelocity)) {
      pivotTalonFX.setControl(pivotHomingRequest);
      return false;
    }
    pivotTalonFX.setPosition(ArmConstants.PIVOT_EXPECTED_HOMING_ZERO);
    pivotTalonFX.setControl(pivotIdleRequest);
    return true;
  }

  @Override
  public boolean nearSetpoint() {
    return pivotPositionSignal.isNear(
        pivotPositionRequest.getPositionMeasure(), ArmConstants.PIVOT_POSITION_TOLERANCE);
  }

  private boolean atUpperHomingHardstop(
      Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    return pivotCurrentSignal.isNear(
            ArmConstants.PIVOT_HOMING_CURRENT, ArmConstants.PIVOT_HOMING_CURRENT_TOLERANCE)
        && pivotVelocitySignal.isNear(
            ArmConstants.PIVOT_HOMING_VELOCITY, ArmConstants.PIVOT_HOMING_VELOCITY_TOLERANCE);
  }
}
