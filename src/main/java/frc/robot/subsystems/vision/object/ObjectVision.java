package frc.robot.subsystems.vision.object;

import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.cameras.CameraConfig;
import frc.robot.util.loggedimpl.LoggedSubsystem;

public class ObjectVision extends LoggedSubsystem {

  public ObjectVision(CameraConfig... cameraConfigs) {
    super(VisionConstants.ObjectVision.LOG_PATH, null, true);
  }
}
