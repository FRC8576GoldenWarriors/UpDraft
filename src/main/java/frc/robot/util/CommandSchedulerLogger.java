package frc.robot.util;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class CommandSchedulerLogger {

  private static CommandSchedulerLogger instance;
  private final HashMap<String, ArrayList<String>> commandSchedularMap;
  private boolean binded = false;

  private static final String LOG_PATH = "CommandSchedular/";

  private static final String INITILIZE_KEY = "Initializing";
  private static final String EXECUTE_KEY = "Executing";
  private static final String FINISH_KEY = "Finished";
  private static final String INTERRUPT_KEY = "Interupted";

  private CommandSchedulerLogger() {
    commandSchedularMap = new HashMap<>();
  }

  public static CommandSchedulerLogger getInstance() {
    if (instance == null) {
      return instance = new CommandSchedulerLogger();
    }
    return instance;
  }

  public void bindCommandScheduler() {

    commandSchedularMap.put(INITILIZE_KEY, new ArrayList<>());
    commandSchedularMap.put(EXECUTE_KEY, new ArrayList<>());
    commandSchedularMap.put(FINISH_KEY, new ArrayList<>());
    commandSchedularMap.put(INTERRUPT_KEY, new ArrayList<>());

    var schedular = CommandScheduler.getInstance();

    schedular.onCommandInitialize(
        command -> {
          commandSchedularMap.get(INITILIZE_KEY).add(command.getName());
        });
    schedular.onCommandExecute(
        command -> {
          commandSchedularMap.get(EXECUTE_KEY).add(command.getName());
        });
    schedular.onCommandFinish(
        command -> {
          commandSchedularMap.get(FINISH_KEY).add(command.getName());
        });
    schedular.onCommandInterrupt(
        command -> {
          commandSchedularMap.get(INTERRUPT_KEY).add(command.getName());
        });

    binded = true;
  }

  public void log() {
    if (!binded) return;

    var initializeList = commandSchedularMap.get(INITILIZE_KEY);
    var executeList = commandSchedularMap.get(EXECUTE_KEY);
    var finishList = commandSchedularMap.get(FINISH_KEY);
    var interruptList = commandSchedularMap.get(INTERRUPT_KEY);

    Logger.recordOutput(LOG_PATH + INITILIZE_KEY, initializeList.toArray(String[]::new));
    Logger.recordOutput(LOG_PATH + EXECUTE_KEY, executeList.toArray(String[]::new));
    Logger.recordOutput(LOG_PATH + FINISH_KEY, finishList.toArray(String[]::new));
    Logger.recordOutput(LOG_PATH + INTERRUPT_KEY, interruptList.toArray(String[]::new));

    initializeList.clear();
    executeList.clear();
    finishList.clear();
    interruptList.clear();
  }
}
