package frc.robot.util.loggedtunable;

import java.util.function.Consumer;

public class LoggedTunablePIDFConstants implements LoggedTunable {

  private static final String[] pidfKeys = {"KP", "KI", "KD", "KS", "KG", "KV", "KA"};
  private static final String logKey = "PID/";
  private final Consumer<Double[]> onAnyChangeAction;
  private final boolean isSubsystemTunable;

  private final LoggedTunableDouble[] loggedTunablePIDFValues;

  public LoggedTunablePIDFConstants(
      String dashboardKey,
      boolean tuningMode,
      Consumer<Double[]> onAnyChangeAction,
      double[] kPIDSGVA) {

    this.onAnyChangeAction = onAnyChangeAction;
    this.isSubsystemTunable = tuningMode;
    loggedTunablePIDFValues = new LoggedTunableDouble[7];

    if (kPIDSGVA.length < 7) {
      throw new IllegalArgumentException(
          "Number of provided PIDSGVA values does not match! Make sure you are passing 7 values");
    }

    for (int i = 0; i < loggedTunablePIDFValues.length; i++) {
      loggedTunablePIDFValues[i] =
          new LoggedTunableDouble(
              dashboardKey + "/" + logKey + "/" + pidfKeys[i],
              kPIDSGVA[i],
              (value) -> {},
              tuningMode);
    }

    if (tuningMode) {
      LoggedTunableUtil.registerLoggedTunable(this);
    }
  }

  @Override
  public void checkForChange() {
    if (!isSubsystemTunable) return;
    // This automatically fires the correct Consumer for whichever specific value changed
    for (LoggedTunableDouble tunable : loggedTunablePIDFValues) {
      if (tunable.hasChanged()) {
        onAnyChangeAction.accept(getPIDSGVA());
      }
    }
  }

  // --- Getters ---

  public double getP() {
    return loggedTunablePIDFValues[0].get();
  }

  public double getI() {
    return loggedTunablePIDFValues[1].get();
  }

  public double getD() {
    return loggedTunablePIDFValues[2].get();
  }

  public double getS() {
    return loggedTunablePIDFValues[3].get();
  }

  public double getG() {
    return loggedTunablePIDFValues[4].get();
  }

  public double getV() {
    return loggedTunablePIDFValues[5].get();
  }

  public double getA() {
    return loggedTunablePIDFValues[6].get();
  }

  public Double[] getPIDSGVA() {
    return new Double[] {getP(), getI(), getD(), getS(), getG(), getV(), getA()};
  }
}
