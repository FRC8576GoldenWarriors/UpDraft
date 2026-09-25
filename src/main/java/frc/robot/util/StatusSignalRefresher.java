package frc.robot.util;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class StatusSignalRefresher {

  private static StatusSignalRefresher instance = null;

  private ArrayList<StatusSignal<?>> signalList;

  private StatusSignal<?>[] statusSignalArray;

  private boolean signalsFinalized;

  private ArrayList<String> failedSignalNames;

  private String logPath = "StatusSignalRefresher/refreshInfo/";

  public static StatusSignalRefresher getInstance() {
    if (instance == null) return instance = new StatusSignalRefresher();
    return instance;
  }

  private StatusSignalRefresher() {
    signalList = new ArrayList<>();
    statusSignalArray = new StatusSignal[0];
    failedSignalNames = new ArrayList<>();
    signalsFinalized = false;
  }

  public void addStatusSignals(StatusSignal<?>... statusSignals) {
    signalList.addAll(Arrays.stream(statusSignals).toList());
  }

  public void addStatusSignals(StatusSignal<?>[]... statusSignals) {
    for (int i = 0; i < statusSignals.length; i++) {
      for (int j = 0; j < statusSignals[i].length; j++) {
        signalList.add(statusSignals[i][j]);
      }
    }
  }

  public void finalizeStatusSignals() {
    if (signalsFinalized) return;

    statusSignalArray =
        signalList.stream()
            .filter(signal -> signal.getAppliedUpdateFrequencyMeasure().gt(Hertz.zero()))
            .toList()
            .toArray(new StatusSignal[0]);

    signalList.clear();

    signalsFinalized = !signalsFinalized;
  }

  public void refreshStatusSignals() {
    BaseStatusSignal.refreshAll(statusSignalArray);
  }

  // Returns false if greater than one status signal has an error
  public boolean checkStatusSignals() {
    failedSignalNames.clear();
    int totalAmountOfSignals = statusSignalArray.length;
    int failedSignalCount = 0;
    int staleSignalCount = 0;

    for (int i = 0; i < totalAmountOfSignals; i++) {
      var signal = statusSignalArray[i];
      if (signal.getStatus().isError()) {
        failedSignalNames.add(signal.getName());
        failedSignalCount++;
      }
      if (signal.getStatus() == StatusCode.CanMessageStale) {
        staleSignalCount++;
      }
    }

    Logger.recordOutput(logPath + "totalAmountOfSignals", totalAmountOfSignals);
    Logger.recordOutput(logPath + "failedSignalCount", failedSignalCount);
    Logger.recordOutput(
        logPath + "successfulSignalCount", totalAmountOfSignals - failedSignalCount);
    Logger.recordOutput(
        logPath + "successfulSignalPercent",
        Percent.of((totalAmountOfSignals - failedSignalCount) / totalAmountOfSignals));
    Logger.recordOutput(
        logPath + "failedSignalPercent",
        Percent.of((double) failedSignalCount / totalAmountOfSignals));
    Logger.recordOutput(
        logPath + "failedSignalNames",
        failedSignalNames.toArray(new String[failedSignalNames.size()]));
    Logger.recordOutput(logPath + "haveSignalsFailed", failedSignalCount > 0);
    Logger.recordOutput(logPath + "haveStaleSignalData", staleSignalCount > 0);
    Logger.recordOutput(
        logPath + "staleSignalPercent",
        Percent.of((double) staleSignalCount / totalAmountOfSignals));
    return failedSignalCount > 0 || staleSignalCount > 0;
  }
}
