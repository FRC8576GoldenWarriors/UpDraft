package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunablePIDFNumbers;
import frc.robot.util.LoggedTunableUtil;
import frc.robot.util.StatusSignalRefresher;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakePivotTalonFX;
  private final TalonFX intakeRollerTalonFX;

  private final MotionMagicVoltage intakePivotPositionRequest =
      new MotionMagicVoltage(Radians.zero());
  private final DutyCycleOut intakePivotHomingRequest = new DutyCycleOut(0.05);
  private final DutyCycleOut intakeRollerDutyCycleRequest = new DutyCycleOut(0);
  private final EmptyControl intakeIdleRequest = new EmptyControl();

  private final LoggedTunablePIDFNumbers intakePivotLoggedTunablePIDF;

  // Status Signals
  private final BooleanSupplier intakePivotConnectedSignal;
  private final StatusSignal<Angle> intakePivotPositionSignal;
  private final StatusSignal<AngularVelocity> intakePivotVelocitySignal;
  private final StatusSignal<Voltage> intakePivotVoltageSignal;
  private final StatusSignal<Current> intakePivotCurrentSignal;

  private final BooleanSupplier intakeRollerConnectedSignal;
  private final StatusSignal<AngularVelocity> intakeRollerVelocitySignal;
  private final StatusSignal<Voltage> intakeRollerVoltageSignal;
  private final StatusSignal<Current> intakeRollerCurrentSignal;

  public IntakeIOTalonFX() {
    intakePivotTalonFX = new TalonFX(IntakeConstants.INTAKE_PIVOT_MOTOR_ID);
    intakeRollerTalonFX = new TalonFX(IntakeConstants.INTAKE_ROLLER_MOTOR_ID);

    intakePivotLoggedTunablePIDF = new LoggedTunablePIDFNumbers(
      "IntakePivot", 
      Arrays.asList(
        IntakeConstants.KP, 
        IntakeConstants.KI, 
        IntakeConstants.KD, 
        IntakeConstants.KS, 
        IntakeConstants.KV, 
        IntakeConstants.KG), 
        Arrays.asList(
          (value) -> {
            IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0.withKP(value);
            this.intakePivotTalonFX.getConfigurator().apply(IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0);
          },
          (value) -> {
            IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0.withKI(value);
            this.intakePivotTalonFX.getConfigurator().apply(IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0);
          },
          (value) -> {
            IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0.withKD(value);
            this.intakePivotTalonFX.getConfigurator().apply(IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0);
          },
          (value) -> {
            IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0.withKS(value);
            this.intakePivotTalonFX.getConfigurator().apply(IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0);
          },
          (value) -> {
            IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0.withKV(value);
            this.intakePivotTalonFX.getConfigurator().apply(IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0);
          },
          (value) -> {
            IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0.withKG(value);
            this.intakePivotTalonFX.getConfigurator().apply(IntakeConstants.INTAKE_PIVOT_CONFIG.Slot0);
          }
        ), 
        IntakeConstants.INTAKE_TUNING_MODE_ENABLED);

    intakePivotConnectedSignal = () -> intakePivotTalonFX.isConnected();
    intakePivotPositionSignal = intakePivotTalonFX.getPosition();
    intakePivotVelocitySignal = intakePivotTalonFX.getVelocity();
    intakePivotVoltageSignal = intakePivotTalonFX.getMotorVoltage();
    intakePivotCurrentSignal = intakePivotTalonFX.getStatorCurrent();

    intakeRollerConnectedSignal = () -> intakeRollerTalonFX.isConnected();
    intakeRollerVelocitySignal = intakeRollerTalonFX.getVelocity();
    intakeRollerVoltageSignal = intakeRollerTalonFX.getMotorVoltage();
    intakeRollerCurrentSignal = intakeRollerTalonFX.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
        intakePivotPositionSignal,
        intakePivotVelocitySignal,
        intakePivotVoltageSignal,
        intakePivotCurrentSignal,
        intakeRollerVelocitySignal,
        intakeRollerVoltageSignal,
        intakeRollerCurrentSignal);

    StatusSignalRefresher.getInstance()
        .addStatusSignals(
            intakePivotPositionSignal,
            intakePivotVelocitySignal,
            intakePivotVoltageSignal,
            intakePivotCurrentSignal,
            intakeRollerVelocitySignal,
            intakeRollerVoltageSignal,
            intakeRollerCurrentSignal);

    intakePivotTalonFX.getConfigurator().apply(IntakeConstants.INTAKE_PIVOT_CONFIG);
    intakeRollerTalonFX.getConfigurator().apply(IntakeConstants.INTAKE_ROLLER_CONFIG);
    LoggedTunableUtil.registerLoggedTunable(intakePivotLoggedTunablePIDF);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakePivotConnected = intakePivotConnectedSignal.getAsBoolean();
    inputs.intakePivotPosition = intakePivotPositionSignal.getValue();
    inputs.intakePivotVelocity = intakePivotVelocitySignal.getValue();
    inputs.intakePivotVoltage = intakePivotVoltageSignal.getValue();
    inputs.intakePivotCurrent = intakePivotCurrentSignal.getValue();

    inputs.intakeRollerConnected = intakeRollerConnectedSignal.getAsBoolean();
    inputs.intakeRollerVelocity = intakeRollerVelocitySignal.getValue();
    inputs.intakeRollerVoltage = intakeRollerVoltageSignal.getValue();
    inputs.intakeRollerCurrent = intakeRollerCurrentSignal.getValue();
  }

  @Override
  public void setWantedIntakePosition(Angle position) {
    intakePivotTalonFX.setControl(intakePivotPositionRequest.withPosition(position));
  }

  @Override
  public void setWantedIntakeRollerSpeed(double dutyCycleOutput) {
    intakeRollerTalonFX.setControl(intakeRollerDutyCycleRequest.withOutput(dutyCycleOutput));
  }

  @Override
  public void idle() {
    intakePivotTalonFX.setControl(intakeIdleRequest);
    intakeRollerTalonFX.setControl(intakeIdleRequest);
  }

  @Override
  public boolean home(Current activeHomingCurrent) {
    if (!atUpperHardstopCurrent(activeHomingCurrent)) {
      intakePivotTalonFX.setControl(intakePivotHomingRequest);
      return false;
    }
    intakePivotTalonFX.setPosition(IntakeConstants.INTAKE_EXPECTED_HOMING_ZERO);
    return true;
  }

  private boolean atUpperHardstopCurrent(Current activeHomingCurrent) {
    return MathUtil.isNear(
        IntakeConstants.PIVOT_HOMING_CURRENT.in(Amps),
        activeHomingCurrent.in(Amps),
        IntakeConstants.PIVOT_HOMING_CURRENT_TOLERANCE.in(Amps));
  }
}
