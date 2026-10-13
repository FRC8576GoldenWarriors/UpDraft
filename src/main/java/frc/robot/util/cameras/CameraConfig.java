package frc.robot.util.cameras;

import edu.wpi.first.math.geometry.Pose3d;

public class CameraConfig {

  private static int cameraInstance = 0;

  public enum CameraType {
    TAG,
    OBJECT
  }

  private final String name;
  private CameraType cameraType = CameraType.TAG;
  private Pose3d robotRelativeLimelightPose = Pose3d.kZero;

  private final int enabledPipelineID = 0;
  private final int disabledPipelineID = 1;

  public CameraConfig(String name) {
    this.name = name;
    cameraInstance++;
  }

  public CameraConfig() {
    this(Integer.toString(cameraInstance));
  }

  public String getLimelightName() {
    return this.name;
  }

  public void withLimelightPose(Pose3d pose) {
    this.robotRelativeLimelightPose = pose;
  }

  public Pose3d getLimelightPose() {
    return this.robotRelativeLimelightPose;
  }

  public void withCameraType(CameraType cameraType) {
    this.cameraType = cameraType;
  }

  public CameraType getCameraType() {
    return this.cameraType;
  }

  public int getEnabledPipelineID() {
    return this.enabledPipelineID;
  }

  public int getDisabledPipelineID() {
    return this.disabledPipelineID;
  }

  public static int getInstanceCount() {
    return cameraInstance;
  }
}
