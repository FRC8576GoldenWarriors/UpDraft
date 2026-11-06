package frc.robot.util.loggedimpl;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class LoggedSysIdRoutine extends SysIdRoutine {

  private static final Velocity<VoltageUnit> VOLTS_PER_SECOND = Volts.per(Seconds).of(0.5);
  private static final Voltage VOLTAGE_STEP = Volts.of(4);
  private static final Time TIMEOUT = Seconds.of(6);

  private List<BooleanSupplier> endConditions = new ArrayList<>();

  public LoggedSysIdRoutine(
      Velocity<VoltageUnit> voltageRampRate,
      Voltage voltageStep,
      Time timeoutTime,
      LoggedSubsystem subsystem) {
    super(
        new SysIdRoutine.Config(
            voltageRampRate,
            voltageStep,
            timeoutTime,
            (state) -> subsystem.record("SysIdState-" + subsystem.getName(), state.toString())),
        new SysIdRoutine.Mechanism(subsystem::voltageIn, null, subsystem));
  }

  public LoggedSysIdRoutine(LoggedSubsystem subsystem) {
    this(VOLTS_PER_SECOND, VOLTAGE_STEP, TIMEOUT, subsystem);
  }

  public Command getForwardDynamicCommand() {
    Command forwardDynamicCommand = this.dynamic(SysIdRoutine.Direction.kForward);
    for (BooleanSupplier endCondition : endConditions) forwardDynamicCommand.until(endCondition);
    return forwardDynamicCommand;
  }

  public Command getBackwardDynamicCommand() {
    Command backwardDynamicCommand = this.dynamic(SysIdRoutine.Direction.kReverse);
    for (BooleanSupplier endCondition : endConditions) backwardDynamicCommand.until(endCondition);
    return backwardDynamicCommand;
  }

  public Command getForwardQuasistaticCommand() {
    Command forwardQuasistaticCommand = this.quasistatic(SysIdRoutine.Direction.kForward);
    for (BooleanSupplier endCondition : endConditions)
      forwardQuasistaticCommand.until(endCondition);
    return forwardQuasistaticCommand;
  }

  public Command getBackwardQuasistaticCommand() {
    Command backwardQuasistaticCommand = this.quasistatic(SysIdRoutine.Direction.kReverse);
    for (BooleanSupplier endCondition : endConditions)
      backwardQuasistaticCommand.until(endCondition);
    return backwardQuasistaticCommand;
  }

  public SysIdRoutine withEndCondition(BooleanSupplier endCondition) {
    this.endConditions.add(endCondition);
    return this;
  }
}
