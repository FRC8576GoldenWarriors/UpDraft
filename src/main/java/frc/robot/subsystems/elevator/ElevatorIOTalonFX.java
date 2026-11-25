package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.generated.TunerConstants;
import frc.robot.util.StatusSignalRefresher;
import frc.robot.util.alertmanager.AlertManager;
import java.util.HashSet;
import java.util.function.BooleanSupplier;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX leftLiftMotor;
  private final TalonFX rightLiftMotor;

  private final MotionMagicVoltage liftPositionRequest = new MotionMagicVoltage(Rotations.zero());
  private final VoltageOut liftHomingPositionRequest = new VoltageOut(ElevatorConstants.LIFT_HOMING_VOLTAGE);
  private final EmptyControl liftIdleRequest = new EmptyControl();
  private final Follower liftFollowerRequest =
      new Follower(ElevatorConstants.LEFT_LIFT_MOTOR_ID, MotorAlignmentValue.Opposed);

  private final BooleanSupplier leftMotorConnectedSignal;
  private final StatusSignal<Voltage> leftMotorVoltageSignal;
  private final StatusSignal<Current> leftMotorStatorCurrentSignal;
  private final StatusSignal<Current> leftMotorSupplyCurrentSignal;

  private final BooleanSupplier rightMotorConnectedSignal;
  private final StatusSignal<Voltage> rightMotorVoltageSignal;
  private final StatusSignal<Current> rightMotorStatorCurrentSignal;
  private final StatusSignal<Current> rightMotorSupplyCurrentSignal;

  private final StatusSignal<Angle> rotationalPositionSignal;
  private final StatusSignal<AngularVelocity> rotationalVelocitySignal;

  private final HashSet<StatusSignal<Current>> liftCurrentSignals = new HashSet<>();

  public ElevatorIOTalonFX() {
    leftLiftMotor = new TalonFX(ElevatorConstants.LEFT_LIFT_MOTOR_ID, TunerConstants.kCANBus);
    rightLiftMotor = new TalonFX(ElevatorConstants.RIGHT_LIFT_MOTOR_ID, TunerConstants.kCANBus);

    leftMotorConnectedSignal = () -> leftLiftMotor.isConnected();
    leftMotorVoltageSignal = leftLiftMotor.getMotorVoltage();
    leftMotorStatorCurrentSignal = leftLiftMotor.getStatorCurrent();
    leftMotorSupplyCurrentSignal = leftLiftMotor.getSupplyCurrent();

    liftCurrentSignals.add(leftMotorSupplyCurrentSignal);

    AlertManager.getInstance()
        .registerAlert(
            new Alert("The Left Lift Motor is Disconnected", AlertType.kError),
            leftMotorConnectedSignal);

    rightMotorConnectedSignal = () -> rightLiftMotor.isConnected();
    rightMotorVoltageSignal = rightLiftMotor.getMotorVoltage();
    rightMotorStatorCurrentSignal = rightLiftMotor.getStatorCurrent();
    rightMotorSupplyCurrentSignal = rightLiftMotor.getSupplyCurrent();

    liftCurrentSignals.add(rightMotorSupplyCurrentSignal);

    AlertManager.getInstance()
        .registerAlert(
            new Alert("The Right Lift Motor is Disconnected", AlertType.kError),
            rightMotorConnectedSignal);

    rotationalPositionSignal = leftLiftMotor.getPosition();
    rotationalVelocitySignal = leftLiftMotor.getVelocity();

    leftLiftMotor.optimizeBusUtilization(Hertz.zero());
    rightLiftMotor.optimizeBusUtilization(Hertz.zero());

    BaseStatusSignal.setUpdateFrequencyForAll(
        ElevatorConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
        leftMotorVoltageSignal,
        leftMotorStatorCurrentSignal,
        leftMotorSupplyCurrentSignal,
        rightMotorVoltageSignal,
        rightMotorStatorCurrentSignal,
        rightMotorSupplyCurrentSignal,
        rotationalPositionSignal,
        rotationalVelocitySignal);

    StatusSignalRefresher.getInstance()
        .addStatusSignals(
            leftMotorVoltageSignal,
            leftMotorStatorCurrentSignal,
            leftMotorSupplyCurrentSignal,
            rightMotorVoltageSignal,
            rightMotorStatorCurrentSignal,
            rightMotorSupplyCurrentSignal,
            rotationalPositionSignal,
            rotationalVelocitySignal);

    leftLiftMotor.getConfigurator().apply(ElevatorConstants.LEFT_LIFT_CONFIG);
    rightLiftMotor.getConfigurator().apply(ElevatorConstants.RIGHT_LIFT_CONFIG);

    TalonFX m1 = new TalonFX(20, TunerConstants.kCANBus);
    TalonFX m2 = new TalonFX(21, TunerConstants.kCANBus);
    TalonFX m3 = new TalonFX(25, TunerConstants.kCANBus);
    TalonFX m4 = new TalonFX(26, TunerConstants.kCANBus);
    CANrange c1 = new CANrange(30, TunerConstants.kCANBus);
    CANrange c2 = new CANrange(31, TunerConstants.kCANBus);
    TalonFX m5 = new TalonFX(40, TunerConstants.kCANBus);
    TalonFX m6 = new TalonFX(45, TunerConstants.kCANBus);

    TalonFX.optimizeBusUtilizationForAll(Hertz.zero(), m1, m2, m3, m4, m5, m6, c1, c2);

    m1.close();
    m2.close();
    m3.close();
    m4.close();
    m5.close();
    m6.close();
    c1.close();
    c2.close();

    rightLiftMotor.setControl(liftFollowerRequest);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected = leftMotorConnectedSignal.getAsBoolean();
    inputs.leftMotorVoltage = leftMotorVoltageSignal.getValue();
    inputs.leftMotorStatorCurrent = leftMotorStatorCurrentSignal.getValue();
    inputs.leftMotorSupplyCurrent = leftMotorSupplyCurrentSignal.getValue();

    inputs.rightMotorConnected = rightMotorConnectedSignal.getAsBoolean();
    inputs.rightMotorVoltage = rightMotorVoltageSignal.getValue();
    inputs.rightMotorStatorCurrent = rightMotorStatorCurrentSignal.getValue();
    inputs.rightMotorSupplyCurrent = rightMotorSupplyCurrentSignal.getValue();

    inputs.rotationalPosition = rotationalPositionSignal.getValue();
    inputs.rotationalVelocity = rotationalVelocitySignal.getValue();

    inputs.linearPosition =
        Meters.of(
            inputs.rotationalPosition.in(Rotations)
                * ElevatorConstants.LINEAR_TO_ANGULAR.in(Value));
    inputs.linearVelocity =
        MetersPerSecond.of(
            inputs.rotationalVelocity.in(RotationsPerSecond)
                * ElevatorConstants.LINEAR_TO_ANGULAR.in(Value));
  }

  @Override
  public void idle() {
    leftLiftMotor.setControl(liftIdleRequest);
  }

  @Override
  public StatusSignal<Current>[] getCurrentSignals() {
    return liftCurrentSignals.toArray(StatusSignal[]::new);
  }

  @Override
  public void setWantedPosition(Angle rotationalPosition) {
    leftLiftMotor.setControl(liftPositionRequest.withPosition(rotationalPosition));
  }

  @Override
  public void setWantedPosition(Distance linearPosition) {
    setWantedPosition(
        Rotations.of(linearPosition.in(Meters) / ElevatorConstants.LINEAR_TO_ANGULAR.in(Value)));
  }

  @Override
  public boolean home() {
    if (!atLowerHardstop()) {
      leftLiftMotor.setControl(liftHomingPositionRequest);
      return false;
    }
    leftLiftMotor.setPosition(Rotations.zero());
    rightLiftMotor.setPosition(Rotations.zero());
    this.idle();
    return true;
  }

  @Override
  public boolean nearSetpoint() {
    return rotationalPositionSignal.isNear(
        liftPositionRequest.getPositionMeasure(),
        ElevatorConstants.linearToAngular(ElevatorConstants.LIFT_POSITION_TOLERANCE));
  }

  @Override
  public void zero() {
    leftLiftMotor.setPosition(0);
    rightLiftMotor.setPosition(0);
  }

  private boolean atLowerHardstop() {
    return leftMotorStatorCurrentSignal.getValue().gt(ElevatorConstants.LIFT_HOMING_CURRENT)
        && rotationalVelocitySignal.isNear(
            ElevatorConstants.linearToAngular(ElevatorConstants.LIFT_HOMING_VELOCITY),
            ElevatorConstants.linearToAngular(ElevatorConstants.LIFT_HOMING_VELOCITY_TOLERANCE));
  }
}
