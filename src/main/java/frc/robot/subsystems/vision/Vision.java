package frc.robot.subsystems.vision;

import frc.robot.util.loggedimpl.LoggedSubsystem;

public class Vision extends LoggedSubsystem {

  public Vision(VisionIO io) {
    super(VisionConstants.LOG_PATH, null, true);
  }
}
