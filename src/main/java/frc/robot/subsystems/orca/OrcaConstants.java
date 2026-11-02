package frc.robot.subsystems.orca;

import frc.robot.subsystems.orca.Orca.WantedState;
import frc.robot.subsystems.swerve.Swerve;

public class OrcaConstants {

  public static String LOG_PATH = "Orca/";

  public static final boolean USE_SYS_ID_MODE = true;
  public static final WantedState WANTED_SYS_ID_STATE = WantedState.SYS_ID_TRANSLATION;

  public static final Swerve.WantedState WANTED_SYS_ID_STATE_SWERVE =
      switch (WANTED_SYS_ID_STATE) {
        case SYS_ID_ROTATION -> Swerve.WantedState.SYS_ID_ROTATION;
        case SYS_ID_STEER -> Swerve.WantedState.SYS_ID_STEER;
        case SYS_ID_TRANSLATION -> Swerve.WantedState.SYS_ID_TRANSLATION;
        default -> Swerve.WantedState.IDLE;
      };

  public static final boolean USE_TUNING_MODE = false;
}
