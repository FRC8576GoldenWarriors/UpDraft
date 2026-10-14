package frc.robot.subsystems.vision.apriltag;

import frc.robot.subsystems.vision.Vision.PoseEstimateConsumer;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.cameras.CameraConfig;
import frc.robot.util.loggedimpl.LoggedSubsystem;

public class ApriltagVision extends LoggedSubsystem {

  private final PoseEstimateConsumer poseEstimateConsumer;
  private final CameraConfig[] cameraConfigs;

  public ApriltagVision(PoseEstimateConsumer poseEstimateConsumer, CameraConfig... cameraConfigs) {
    super(VisionConstants.AprilTagVision.LOG_PATH, null, true);
    this.poseEstimateConsumer = poseEstimateConsumer;
    this.cameraConfigs = cameraConfigs;
  }
}
