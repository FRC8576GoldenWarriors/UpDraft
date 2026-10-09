package frc.robot.util.loggedtunable;

import edu.wpi.first.math.MathUtil;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableDouble implements DoubleSupplier, LoggedTunable {

  private double currentValue;
  private double lastValue;
  private String logKey = "/Tuning";

  private final Consumer<Double> onChangeAction;
  private final boolean subsystemTuningMode;

  private LoggedNetworkNumber loggedNumber;

  public LoggedTunableDouble(
      String dashboardKey,
      double defaultValue,
      Consumer<Double> onChangeAction,
      boolean subsystemTuningMode) {
    this.logKey += "/" + dashboardKey;
    this.currentValue = defaultValue;
    this.lastValue = currentValue;
    this.onChangeAction = onChangeAction;
    this.subsystemTuningMode = subsystemTuningMode;
    initLoggedNetworkNumber();
    this.onChangeAction.accept(currentValue);
    if (subsystemTuningMode) {
      LoggedTunableUtil.registerLoggedTunable(this);
    }
  }

  public LoggedTunableDouble(
      String dashboardKey, Consumer<Double> onChangeAction, boolean subsystemTuningMode) {
    this(dashboardKey, 0, onChangeAction, subsystemTuningMode);
  }

  private void initLoggedNetworkNumber() {
    if (subsystemTuningMode) {
      loggedNumber = new LoggedNetworkNumber(this.logKey, this.currentValue);
    }
  }

  public double get() {
    return this.subsystemTuningMode ? loggedNumber.get() : currentValue;
  }

  public boolean hasChanged() {
    currentValue = get();
    if (!MathUtil.isNear(lastValue, currentValue, 1e-6)) {
      lastValue = currentValue;
      return true;
    }
    return false;
  }

  public void checkForChange() {
    if (!subsystemTuningMode) return;
    if (hasChanged()) {
      onChangeAction.accept(get());
    }
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
