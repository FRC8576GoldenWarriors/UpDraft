package frc.robot.util;

import java.util.ArrayList;

public class LoggedTunableUtil {
    private static ArrayList<LoggedTunable> loggedTunables = new ArrayList<>();

    public static void registerLoggedTunable(LoggedTunable loggedTunable) {
        loggedTunables.add(loggedTunable);
    }

    public static void checkLoggedTunables() {
        for(LoggedTunable lt : loggedTunables) {
            lt.checkForChange();
        }
    }
}

interface LoggedTunable {
    default void checkForChange() {};
}
