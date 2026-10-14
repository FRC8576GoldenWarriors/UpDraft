package frc.robot.subsystems.vision;

import frc.robot.util.cameras.CameraConfig;
import frc.robot.util.loggedimpl.LoggedSubsystem;

public class Vision extends LoggedSubsystem {

  public Vision(CameraConfig... cameraConfigs) {
    super(VisionConstants.LOG_PATH, null, true);
  }
}
