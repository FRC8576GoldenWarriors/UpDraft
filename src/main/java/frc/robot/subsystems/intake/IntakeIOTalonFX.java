package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import frc.robot.generated.TunerConstants;
import frc.robot.util.StatusSignalRefresher;
import frc.robot.util.loggedtunable.LoggedTunablePIDFConstants;
import java.util.function.BooleanSupplier;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakePivotTalonFX;
  private final TalonFX intakeRollerTalonFX;

  private final MotionMagicVoltage intakePivotPositionRequest =
      new MotionMagicVoltage(Radians.zero());
  private final DutyCycleOut intakePivotHomingRequest = new DutyCycleOut(IntakeConstants.PIVOT_HOMING_DUTY_CYCLE);
  private final DutyCycleOut intakeRollerDutyCycleRequest = new DutyCycleOut(0);
  private final EmptyControl intakeIdleRequest = new EmptyControl();

  private final LoggedTunablePIDFConstants intakePivotLoggedTunablePIDFConstants;

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
    intakePivotTalonFX = new TalonFX(IntakeConstants.INTAKE_PIVOT_MOTOR_ID, TunerConstants.kCANBus);
    intakeRollerTalonFX =
        new TalonFX(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, TunerConstants.kCANBus);

    intakePivotLoggedTunablePIDFConstants =
        new LoggedTunablePIDFConstants(
            IntakeConstants.LOG_PATH_INTAKE_PIVOT,
            IntakeConstants.INTAKE_TUNING_MODE_ENABLED,
            (values) -> {
              intakePivotTalonFX
                  .getConfigurator()
                  .apply(
                      IntakeConstants.INTAKE_PIVOT_CONFIG
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
  public boolean home(Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    if (!atUpperHardstop(activeHomingCurrent, activeHomingVelocity)) {
      intakePivotTalonFX.setControl(intakePivotHomingRequest);
      return false;
    }
    intakePivotTalonFX.setPosition(IntakeConstants.INTAKE_EXPECTED_HOMING_ZERO);
    return true;
  }

  private boolean atUpperHardstop(
      Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    return MathUtil.isNear(
            IntakeConstants.PIVOT_HOMING_CURRENT.in(Amps),
            activeHomingCurrent.in(Amps),
            IntakeConstants.PIVOT_HOMING_CURRENT_TOLERANCE.in(Amps))
        && MathUtil.isNear(
            IntakeConstants.PIVOT_HOMING_VELOCITY.in(RotationsPerSecond),
            activeHomingVelocity.in(RotationsPerSecond),
            IntakeConstants.PIVOT_HOMING_VELOCITY_TOLERANCE.in(RotationsPerSecond));
  }
}
