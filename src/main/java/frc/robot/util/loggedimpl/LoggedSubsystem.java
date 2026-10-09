package frc.robot.util.loggedimpl;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.loggedtunable.LoggedTunableDouble;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import us.hebi.quickbuf.ProtoMessage;

public class LoggedSubsystem extends SubsystemBase {

  private final String logPath;
  private final HashSet<StatusSignal<Current>> subsystemSupplyCurrentSignals = new HashSet<>();
  private static final CopyOnWriteArraySet<StatusSignal<Current>> allSubsystemSupplyCurrentSignals =
      new CopyOnWriteArraySet<>();

  public LoggedSubsystem(
      String logPath, StatusSignal<Current>[] currentSignals, boolean skipCurrentSignalCheck) {
    if (currentSignals == null) skipCurrentSignalCheck = true;

    if (!skipCurrentSignalCheck && currentSignals.length <= 0)
      throw new IllegalArgumentException(
          logPath.substring(0, logPath.length() - 1)
              + " requires current signals to log. Ensure you are passing appropriate current signals to log. "
              + "Otherwise pass true in the constructor to skip the current signal check.");

    this.logPath = logPath;
    if (!skipCurrentSignalCheck) this.addToPowerLogging(currentSignals);
  }

  public LoggedSubsystem(String logPath, StatusSignal<Current>[] currentSignals) {
    this(logPath, currentSignals, false);
  }

  public LoggedTunableDouble newSubsystemLoggedTunableDouble(
      String dashboardKey, Consumer<Double> onChangeAction, boolean subsystemTuningMode) {
    return new LoggedTunableDouble(logPath + dashboardKey, onChangeAction, subsystemTuningMode);
  }

  private void addToPowerLogging(StatusSignal<Current>[] signals) {
    for (StatusSignal<Current> signal : signals) {
      subsystemSupplyCurrentSignals.add(signal);
    }
    allSubsystemSupplyCurrentSignals.addAll(subsystemSupplyCurrentSignals);
  }

  private void recordPowerData() {
    Current totalSubsystemSupplyCurrent = Amps.zero();
    for (StatusSignal<Current> currentSignal : subsystemSupplyCurrentSignals) {
      totalSubsystemSupplyCurrent = totalSubsystemSupplyCurrent.plus(currentSignal.getValue());
    }

    Current allSubsystemSupplyCurrent = Amps.zero();
    for (StatusSignal<Current> currentSignal : allSubsystemSupplyCurrentSignals) {
      allSubsystemSupplyCurrent = allSubsystemSupplyCurrent.plus(currentSignal.getValue());
    }

    record("PowerMonitor/TotalCurrent", totalSubsystemSupplyCurrent);
    record(
        "PowerMonitor/TotalPowerWatts",
        totalSubsystemSupplyCurrent.times(Volts.of(RobotController.getBatteryVoltage())));
    record(
        "PowerMonitor/TotalEnergyJoules",
        totalSubsystemSupplyCurrent
            .times(Volts.of(RobotController.getBatteryVoltage()))
            .times(Seconds.of(LoggedRobot.defaultPeriodSecs)));

    Logger.recordOutput("PowerMonitor/TotalSupplyCurrent", allSubsystemSupplyCurrent);
    Logger.recordOutput(
        "PowerMonitor/TotalPowerWatts",
        allSubsystemSupplyCurrent.times(Volts.of(RobotController.getBatteryVoltage())));
    Logger.recordOutput(
        "PowerMonitor/TotalEnergyJoules",
        allSubsystemSupplyCurrent
            .times(Volts.of(RobotController.getBatteryVoltage()))
            .times(Seconds.of(LoggedRobot.defaultPeriodSecs)));
  }

  /**
   * Processes a set of inputs, logging them on the real robot or updating them in the simulator.
   * This should be called every loop cycle after updating the inputs from the hardware (if
   * applicable).
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param inputs The inputs to log or update.
   */
  public void processInputs(LoggableInputs inputs) {
    this.processInputs("", inputs);
  }

  /**
   * Processes a set of inputs, logging them on the real robot or updating them in the simulator.
   * This should be called every loop cycle after updating the inputs from the hardware (if
   * applicable).
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param inputs The inputs to log or update.
   */
  public void processInputs(String key, LoggableInputs inputs) {
    Logger.processInputs(logPath + key, inputs);
    this.recordPowerData();
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, byte[] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param value The wanted state to log.
   */
  public <E extends Enum<E> & LoggedWantedState> void recordWantedState(E value) {
    this.record(logPath + "WantedState", value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param value The wanted state to log.
   */
  public <E extends Enum<E> & LoggedSystemState> void recordSystemState(E value) {
    this.record(logPath + "SystemState", value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, byte[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, boolean value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, BooleanSupplier value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, boolean[] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, boolean[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, int value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, IntSupplier value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, int[] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, int[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, long value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, LongSupplier value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, long[] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, long[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, float value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method saves the float value with unit metadata that is compatible with AdvantageScope.
   * The raw value preserves the <b>user-specified unit</b>.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   * @param unit The unit of the value.
   */
  public void record(String key, float value, Unit unit) {
    Logger.recordOutput(logPath + key, value, unit);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method saves the float value with unit metadata that is compatible with AdvantageScope.
   * The raw value preserves the <b>user-specified unit</b>.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   * @param unit The unit of the value.
   */
  public void record(String key, float value, String unit) {
    Logger.recordOutput(logPath + key, value, unit);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, float[] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, float[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, double value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method saves the double value with unit metadata that is compatible with
   * AdvantageScope. The raw value preserves the <b>user-specified unit</b>.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   * @param unit The unit of the value.
   */
  public void record(String key, double value, Unit unit) {
    Logger.recordOutput(logPath + key, value, unit);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method saves the double value with unit metadata that is compatible with
   * AdvantageScope. The raw value preserves the <b>user-specified unit</b>.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, double value, String unit) {
    Logger.recordOutput(logPath + key, value, unit);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, DoubleSupplier value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, double[] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, double[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, String value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, String[] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, String[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param <E> The enum type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <E extends Enum<E>> void record(String key, E value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param <E> The enum type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <E extends Enum<E>> void record(String key, E[] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param <E> The enum type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <E extends Enum<E>> void record(String key, E[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method saves the value with unit metadata that is compatible with AdvantageScope. The
   * raw value preserves the <b>user-specified unit</b>.
   *
   * @param <U> The unit type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <U extends Unit> void record(String key, Measure<U> value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes a single object as a struct.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param <T> The struct type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param struct The struct serialization object.
   * @param value The value of the field.
   */
  public <T> void record(String key, Struct<T> struct, T value) {
    Logger.recordOutput(logPath + key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes an array of objects as a struct.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param <T> The struct type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param struct The struct serialization object.
   * @param value The value of the field.
   */
  @SafeVarargs
  public final <T> void record(String key, Struct<T> struct, T... value) {
    Logger.recordOutput(logPath + key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param <T> The struct type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param struct The struct serialization object.
   * @param value The value of the field.
   */
  public <T> void record(String key, Struct<T> struct, T[][] value) {
    Logger.recordOutput(logPath + key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes a single object as a protobuf. Protobuf should only be used for
   * objects that do not support struct serialization.
   *
   * @param <T> The value type.
   * @param <MessageType> The protobuf message type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param proto The protobuf serialization object.
   * @param value The value of the field.
   */
  public <T, MessageType extends ProtoMessage<?>> void record(
      String key, Protobuf<T, MessageType> proto, T value) {
    Logger.recordOutput(logPath + key, proto, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes a single object as a struct or protobuf automatically. Struct is
   * preferred if both methods are supported.
   *
   * @param <T> The object type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <T extends WPISerializable> void record(String key, T value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes an array of objects as a struct automatically. Top-level protobuf
   * arrays are not supported.
   *
   * @param <T> The object type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  @SafeVarargs
  public final <T extends StructSerializable> void record(String key, T... value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes an array of objects as a struct automatically. Top-level protobuf
   * arrays are not supported.
   *
   * @param <T> The object type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <T extends StructSerializable> void record(String key, T[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes an array of objects as a struct automatically. Top-level protobuf
   * arrays are not supported.
   *
   * @param <R> The record type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <R extends Record> void record(String key, R value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes an array of objects as a struct automatically. Top-level protobuf
   * arrays are not supported.
   *
   * @param <R> The record type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  @SafeVarargs
  public final <R extends Record> void record(String key, R... value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes an array of objects as a struct automatically. Top-level protobuf
   * arrays are not supported.
   *
   * @param <R> The record type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <R extends Record> void record(String key, R[][] value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>The current position of the Mechanism2d is logged once as a set of nested fields. If the
   * position is updated, this method must be called again.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public void record(String key, LoggedMechanism2d value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The color value to record.
   */
  public void record(String key, Color value) {
    Logger.recordOutput(logPath + key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the simulator, use this
   * method to record extra data based on the original inputs.
   *
   * <p>This method serializes an array of objects as a struct automatically. Top-level protobuf
   * arrays are not supported.
   *
   * <p>This method is <b>not thread-safe</b> and should only be called from the main thread. Check
   * the <a href=
   * "https://docs.advantagekit.org/getting-started/common-issues/multithreading">documentation</a>
   * for details.
   *
   * @param <T> The object type.
   * @param key The name of the field to record. It will be stored under the path specified by
   *     {@code logPath}.
   * @param value The value of the field.
   */
  public <T extends StructSerializable> void recordOutput(String key, ArrayList<T> value) {
    Logger.recordOutput(logPath + key, value.toArray(new Pose2d[value.size()]));
  }
}
