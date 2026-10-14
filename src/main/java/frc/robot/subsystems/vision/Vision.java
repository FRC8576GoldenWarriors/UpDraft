package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.object.ObjectVision;
import frc.robot.util.cameras.CameraConfig;
import frc.robot.util.cameras.CameraConfig.CameraType;
import java.util.ArrayList;
import java.util.List;

public class Vision {

  private final PoseEstimateConsumer consumer;
  private final CameraConfig[] configs;

  private final List<CameraConfig> apriltagCameraConfigs = new ArrayList<>();
  private final List<CameraConfig> objectCameraConfigs = new ArrayList<>();

  private final ApriltagVision apriltagVision;
  private final ObjectVision objectVision;

  public Vision(PoseEstimateConsumer consumer, CameraConfig... cameraConfigs) {
    this.consumer = consumer;
    this.configs = cameraConfigs;

    for (CameraConfig config : cameraConfigs) {
      if (config.getCameraType() == CameraType.TAG) apriltagCameraConfigs.add(config);
      if (config.getCameraType() == CameraType.OBJECT) objectCameraConfigs.add(config);
    }

    apriltagVision =
        new ApriltagVision(consumer, apriltagCameraConfigs.toArray(CameraConfig[]::new));
    objectVision = new ObjectVision(objectCameraConfigs.toArray(CameraConfig[]::new));
  }

  @FunctionalInterface
  public static interface PoseEstimateConsumer {
    public void accept(Pose2d robotPoseMeters, Time timestamp, Matrix<N3, N1> standardDeviations);
  }
}
