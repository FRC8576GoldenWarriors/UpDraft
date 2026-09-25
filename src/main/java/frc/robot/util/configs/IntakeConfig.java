package frc.robot.util.configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;

public class IntakeConfig {

  private int intakePivotId;
  private int intakeRollerId;

  private TalonFXConfiguration intakePivotConfiguration;
  private TalonFXConfiguration intakeRollerConfiguration;

  private Frequency intakeStatusSignalUpdateFrequency;

  private Current intakeHomingCurrent;
  private Current intakeHomingCurrentTolerance;

  private Angle intakeHomingExpectedZero;

  private Angle intakeDownPosition;
  private Angle intakeUpPosition;

  private double intakeDeployRollerSpeed;
  private double intakeRetractRollerSpeed;
  private double intakeIntakeRollerSpeed;

  public IntakeConfig() {}

  public IntakeConfig withIntakePivotId(int id) {
    this.intakePivotId = id;
    return this;
  }

  public IntakeConfig withIntakeRollerId(int id) {
    this.intakeRollerId = id;
    return this;
  }

  public IntakeConfig withIntakePivotConfig(TalonFXConfiguration config) {
    this.intakePivotConfiguration = config;
    return this;
  }

  public IntakeConfig withIntakeRollerConfig(TalonFXConfiguration config) {
    this.intakeRollerConfiguration = config;
    return this;
  }

  public IntakeConfig withIntakeStatusSignalUpdateFrequency(Frequency updateFrequency) {
    this.intakeStatusSignalUpdateFrequency = updateFrequency;
    return this;
  }

  public IntakeConfig withIntakeHomingCurrent(Current homingCurrent) {
    this.intakeHomingCurrent = homingCurrent;
    return this;
  }

  public IntakeConfig withIntakeHomingCurrentTolerance(Current homingCurrentTolerance) {
    this.intakeHomingCurrentTolerance = homingCurrentTolerance;
    return this;
  }

  public IntakeConfig withIntakeHomingExpectedZero(Angle expectedZero) {
    this.intakeHomingExpectedZero = expectedZero;
    return this;
  }

  public IntakeConfig withIntakeDownPosition(Angle downPosition) {
    this.intakeDownPosition = downPosition;
    return this;
  }

  public IntakeConfig withIntakeUpPosition(Angle upPosition) {
    this.intakeUpPosition = upPosition;
    return this;
  }

  public IntakeConfig withIntakeDeployRollerSpeed(double dutyCycleOutput) {
    this.intakeDeployRollerSpeed = dutyCycleOutput;
    return this;
  }

  public IntakeConfig withIntakeRetractRollerSpeed(double dutyCycleOutput) {
    this.intakeRetractRollerSpeed = dutyCycleOutput;
    return this;
  }

  public IntakeConfig withIntakeIntakeRollerSpeed(double dutyCycleOutput) {
    this.intakeIntakeRollerSpeed = dutyCycleOutput;
    return this;
  }

  public int getIntakePivotId() {
    return this.intakePivotId;
  }

  public int getIntakeRollerId() {
    return this.intakeRollerId;
  }

  public TalonFXConfiguration getIntakePivotConfiguration() {
    return this.intakePivotConfiguration;
  }

  public TalonFXConfiguration getIntakeRollerConfiguration() {
    return this.intakeRollerConfiguration;
  }

  public Frequency getIntakeStatusSignalUpdateFrequency() {
    return this.intakeStatusSignalUpdateFrequency;
  }

  public Current getIntakeHomingCurrent() {
    return this.intakeHomingCurrent;
  }

  public Current getIntakeHomingCurrentTolerance() {
    return this.intakeHomingCurrentTolerance;
  }

  public Angle getIntakeHomingExpectedZero() {
    return this.intakeHomingExpectedZero;
  }

  public Angle getIntakeDownPosition() {
    return this.intakeDownPosition;
  }

  public Angle getIntakeUpPosition() {
    return this.intakeUpPosition;
  }

  public double getIntakeDeployRollerSpeed() {
    return this.intakeDeployRollerSpeed;
  }

  public double getIntakeRetractRollerSpeed() {
    return this.intakeRetractRollerSpeed;
  }

  public double getIntakeIntakeRollerSpeed() {
    return this.intakeIntakeRollerSpeed;
  }
}
