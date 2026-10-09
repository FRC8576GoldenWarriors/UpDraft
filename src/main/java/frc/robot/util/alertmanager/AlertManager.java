package frc.robot.util.alertmanager;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Alert;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

public class AlertManager {

  private static AlertManager alertManagerInstance;

  private final HashMap<Alert, BooleanSupplier> alertMap;

  private AlertManager() {
    alertMap = new HashMap<>();
  }

  public static AlertManager getInstance() {
    if (alertManagerInstance == null) {
      return alertManagerInstance = new AlertManager();
    }
    return alertManagerInstance;
  }

  public void registerAlert(Alert alert, BooleanSupplier alertState) {
    if (alert == null || alertState == null) return;
    alertMap.put(alert, alertState);
  }

  public void registerAlert(Pair<Alert, BooleanSupplier> alertAndAlertStatePair) {
    if (alertAndAlertStatePair == null) return;
    registerAlert(alertAndAlertStatePair.getFirst(), alertAndAlertStatePair.getSecond());
  }

  public void registerAlert(Pair<Alert, BooleanSupplier>... alertAndAlertStatePairs) {
    for (Pair<Alert, BooleanSupplier> alertAndAlertStatePair : alertAndAlertStatePairs) {
      registerAlert(alertAndAlertStatePair);
    }
  }

  public void periodic() {
    alertMap.forEach(
        (alert, state) -> {
          alert.set(!state.getAsBoolean());
        });
  }
}
