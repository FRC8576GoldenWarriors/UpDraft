package frc.robot.subsystems.vision.object;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionObjectIOLimelight implements VisionObjectIO {
  private final String limelightName;
  private final NetworkTable limelightTable;

  private final IntegerSubscriber hasTargetSubscriber;
  private final DoubleSubscriber horizontalAngleSubscriber;
  private final DoubleSubscriber verticalAngleSubscriber;
  private final DoubleSubscriber visibleTargetAreaSubscriber;
  private final DoubleSubscriber pipelineLatencySubscriber;
  private final DoubleSubscriber captureLatencySubscriber;
  private final DoubleSubscriber isConnectedSubscriber;
  private final StringSubscriber objectClassSubscriber;
  private final DoubleArraySubscriber hardwareDataSubscriber;
  private final DoubleArraySubscriber imuDataSubscriber;
  private final DoubleArraySubscriber rawCornersSubscriber;
  private final DoubleArraySubscriber captureRewindSubscriber;
  private final DoubleArraySubscriber objectDetectionSubscriber;

  private final DoubleArrayPublisher setCameraPoseInRobotSpacePublisher;
  private final DoubleArrayPublisher cropCameraFOVPublisher;
  private final IntegerPublisher setThrottlePublisher;
  private final DoublePublisher enableRewindPublisher;
  private final DoubleArrayPublisher captureRewindPublisher;
  private final IntegerPublisher setIMUModePublisher;
  private final DoublePublisher setIMUAssistPublisher;

  private Optional<double[]> hardwareDataArray = Optional.empty();
  private Optional<double[]> imuDataArray = Optional.empty();
  private Optional<double[]> rawCornerArray = Optional.empty();
  private Optional<double[]> objectDetectionArray = Optional.empty();

  private ArrayList<ObjectDetection> objectDetectionObservations = new ArrayList<>();

  public VisionObjectIOLimelight(String limelightName) {
    this.limelightName = limelightName;
    this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

    this.hasTargetSubscriber = limelightTable.getIntegerTopic("tv").subscribe(0);
    this.horizontalAngleSubscriber = limelightTable.getDoubleTopic("tx").subscribe(0);
    this.verticalAngleSubscriber = limelightTable.getDoubleTopic("ty").subscribe(0);
    this.visibleTargetAreaSubscriber = limelightTable.getDoubleTopic("ta").subscribe(0);
    this.pipelineLatencySubscriber = limelightTable.getDoubleTopic("tl").subscribe(0);
    this.captureLatencySubscriber = limelightTable.getDoubleTopic("cl").subscribe(0);
    this.isConnectedSubscriber = limelightTable.getDoubleTopic("hb").subscribe(0);
    this.objectClassSubscriber = limelightTable.getStringTopic("tdclass").subscribe("None");
    this.hardwareDataSubscriber =
        limelightTable.getDoubleArrayTopic("hw").subscribe(new double[] {});
    this.imuDataSubscriber = limelightTable.getDoubleArrayTopic("imu").subscribe(new double[] {});
    this.rawCornersSubscriber =
        limelightTable.getDoubleArrayTopic("tcornxy").subscribe(new double[] {});
    this.captureRewindSubscriber =
        limelightTable.getDoubleArrayTopic("capture_rewind").subscribe(new double[] {0, 0});
    this.objectDetectionSubscriber =
        limelightTable.getDoubleArrayTopic("rawdetections").subscribe(new double[] {});

    this.setCameraPoseInRobotSpacePublisher =
        limelightTable.getDoubleArrayTopic("camerapose_robotspace").publish();
    this.cropCameraFOVPublisher = limelightTable.getDoubleArrayTopic("crop").publish();
    this.setThrottlePublisher = limelightTable.getIntegerTopic("throttle_set").publish();
    this.enableRewindPublisher = limelightTable.getDoubleTopic("rewind_enable_set").publish();
    this.captureRewindPublisher = limelightTable.getDoubleArrayTopic("capture_rewind").publish();
    this.setIMUModePublisher = limelightTable.getIntegerTopic("imumode_set").publish();
    this.setIMUAssistPublisher = limelightTable.getDoubleTopic("imuassistalpha_set").publish();
  }

  @Override
  public void updateInputs(VisionObjectIOInputs inputs) {
    updateCachedData();

    objectDetectionObservations.clear();

    updateObjectDetections();

    inputs.hasTarget = hasTarget();
    inputs.horizontalAngle = getHorizontalAngle();
    inputs.verticalAngle = getVerticalAngle();
    inputs.visibleTargetArea = getVisibleTargetArea();
    inputs.pipelineLatency = getPipelineLatency();
    inputs.captureLatency = getCaptureLatency();

    double heartbeat = getHeartbeat();
    inputs.isConnected = (inputs.heartbeat != heartbeat);
    inputs.heartbeat = heartbeat;

    inputs.cpuTemperature = getCPUTemperature();
    inputs.cpuUsage = getCPUUsage();
    inputs.ramUsage = getRAMUsage();
    inputs.framesPerSecond = getFPS();

    inputs.objectClass = getObjectClass();

    inputs.suppliedRobotYaw = getSuppliedRobotYaw();
    inputs.limelightRotation3d = getLimelightRotation3d();
    inputs.limelightRotationalVelocity3d = getLimelightRotationalVelocity3d();
    inputs.limelightAcceleration3d = getLimelightAcceleration3d();

    inputs.rawCorners = getRawCorners();

    inputs.objectDetections = objectDetectionObservations.toArray(ObjectDetection[]::new);
  }

  @Override
  public void setCameraPoseInRobotSpace(Pose3d robotRelativeCameraPose3d) {
    double[] cameraPoseRobotspace = new double[6];
    cameraPoseRobotspace[0] = robotRelativeCameraPose3d.getX();
    cameraPoseRobotspace[1] = robotRelativeCameraPose3d.getY();
    cameraPoseRobotspace[2] = robotRelativeCameraPose3d.getZ();

    var cameraRotRobotspace = robotRelativeCameraPose3d.getRotation();

    cameraPoseRobotspace[3] = cameraRotRobotspace.getMeasureX().in(Degrees);
    cameraPoseRobotspace[4] = cameraRotRobotspace.getMeasureY().in(Degrees);
    cameraPoseRobotspace[5] = cameraRotRobotspace.getMeasureZ().in(Degrees);

    setCameraPoseInRobotSpacePublisher.accept(cameraPoseRobotspace);
  }

  @Override
  public void cropCameraFOV(double x0, double x1, double y0, double y1) {
    double[] boundingBox = new double[4];
    boundingBox[0] = Math.min(x0, x1);
    boundingBox[1] = Math.max(x0, x1);
    boundingBox[2] = Math.min(y0, y1);
    boundingBox[3] = Math.max(y0, y1);
    cropCameraFOVPublisher.accept(boundingBox);
  }

  @Override
  public void setThrottle(int throttle) {
    setThrottlePublisher.accept(throttle);
  }

  @Override
  public void enableRewind(boolean enabledRewind) {
    enableRewindPublisher.accept((enabledRewind) ? 1.0 : 0.0);
  }

  @Override
  public void captureRewind(int durationSeconds) {
    var currentRewindStatus = getCaptureRewindData();
    currentRewindStatus[0] += 1;
    currentRewindStatus[1] = Math.min(165, durationSeconds);
    captureRewindPublisher.accept(currentRewindStatus);
  }

  @Override
  public void setIMUMode(IMUMode imuMode) {
    int imuModeInt =
        switch (imuMode) {
          case USE_EXTERNAL_IMU -> 0;
          case USE_EXTERNAL_TO_SEED_INTERNAL_IMU -> 1;
          case USE_INTERNAL_IMU -> 2;
          case USE_INTERNAL_IMU_WITH_MT1_ASSISTANCE -> 3;
          case USE_INTERNAL_IMU_WITH_EXTERNAL_ASSISTANCE -> 4;
          default -> 0;
        };
    setIMUModePublisher.accept(imuModeInt);
  }

  @Override
  public void setIMUAssist(double assist) {
    setIMUAssistPublisher.accept(assist);
  }

  @Override
  public void flush() {
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public String getLimelightName() {
    return limelightName;
  }

  private void updateCachedData() {
    hardwareDataArray = checkDoubleArrayForData(hardwareDataSubscriber.get());
    imuDataArray = checkDoubleArrayForData(imuDataSubscriber.get());
    rawCornerArray = checkDoubleArrayForData(rawCornersSubscriber.get());
    objectDetectionArray = checkDoubleArrayForData(objectDetectionSubscriber.get());
  }

  private Optional<double[]> checkDoubleArrayForData(double[] arrayToCheck) {
    return (arrayToCheck == null || arrayToCheck.length < 1)
        ? Optional.empty()
        : Optional.of(arrayToCheck);
  }

  private boolean hasTarget() {
    return hasTargetSubscriber.get() == 1;
  }

  private Angle getHorizontalAngle() {
    return Degrees.of(horizontalAngleSubscriber.get());
  }

  private Angle getVerticalAngle() {
    return Degrees.of(verticalAngleSubscriber.get());
  }

  private Dimensionless getVisibleTargetArea() {
    return Percent.of(visibleTargetAreaSubscriber.get());
  }

  private Time getPipelineLatency() {
    return Milliseconds.of(pipelineLatencySubscriber.get());
  }

  private Time getCaptureLatency() {
    return Milliseconds.of(captureLatencySubscriber.get());
  }

  private double getHeartbeat() {
    return isConnectedSubscriber.get();
  }

  private Temperature getCPUTemperature() {
    return Celsius.of((hardwareDataArray.isPresent()) ? hardwareDataArray.get()[0] : 0);
  }

  private Dimensionless getCPUUsage() {
    return Percent.of((hardwareDataArray.isPresent()) ? hardwareDataArray.get()[1] : 0);
  }

  private Dimensionless getRAMUsage() {
    return Percent.of((hardwareDataArray.isPresent()) ? hardwareDataArray.get()[2] : 0);
  }

  private int getFPS() {
    return (int) ((hardwareDataArray.isPresent()) ? hardwareDataArray.get()[3] : 0);
  }

  private Angle getSuppliedRobotYaw() {
    return Degrees.of((imuDataArray.isPresent()) ? imuDataArray.get()[0] : 0);
  }

  private String getObjectClass() {
    return objectClassSubscriber.get();
  }

  private Rotation3d getLimelightRotation3d() {
    return (imuDataArray.isPresent())
        ? new Rotation3d(
            Degrees.of(imuDataArray.get()[1]),
            Degrees.of(imuDataArray.get()[2]),
            Degrees.of(imuDataArray.get()[3]))
        : Rotation3d.kZero;
  }

  private Rotation3d getLimelightRotationalVelocity3d() {
    return (imuDataArray.isPresent())
        ? new Rotation3d(
            Degrees.of(imuDataArray.get()[4]),
            Degrees.of(imuDataArray.get()[5]),
            Degrees.of(imuDataArray.get()[6]))
        : Rotation3d.kZero;
  }

  private Translation3d getLimelightAcceleration3d() {
    return (imuDataArray.isPresent())
        ? new Translation3d(
            Meters.of(imuDataArray.get()[7]),
            Meters.of(imuDataArray.get()[8]),
            Meters.of(imuDataArray.get()[9]))
        : Translation3d.kZero;
  }

  private Translation2d[] getRawCorners() {
    List<Translation2d> cornerList = new ArrayList<>();
    var corners = (rawCornerArray.isPresent()) ? rawCornerArray.get() : new double[0];

    for (int i = 0; i < corners.length; i += 2) {
      if (i >= (corners.length - 1)) break;
      cornerList.add(new Translation2d(corners[i], corners[i + 1]));
    }

    return cornerList.toArray(Translation2d[]::new);
  }

  private double[] getCaptureRewindData() {
    return captureRewindSubscriber.get();
  }

  private void updateObjectDetections() {
    if (objectDetectionArray.isEmpty()) return;
    var checkedObjectDetectionArray = objectDetectionArray.get();

    for (int i = 0; i + 11 < checkedObjectDetectionArray.length; i += 12) {
      GamePiece gamePiece = convertIDToGamePiece((int) checkedObjectDetectionArray[i]);
      Angle horizontalAngle = Degrees.of(checkedObjectDetectionArray[i + 1]);
      Angle verticalAngle = Degrees.of(checkedObjectDetectionArray[i + 2]);
      Dimensionless targetArea = Percent.of(checkedObjectDetectionArray[i + 3]);
      Translation2d corner1 =
          new Translation2d(checkedObjectDetectionArray[i + 4], checkedObjectDetectionArray[i + 5]);
      Translation2d corner2 =
          new Translation2d(checkedObjectDetectionArray[i + 6], checkedObjectDetectionArray[i + 7]);
      Translation2d corner3 =
          new Translation2d(checkedObjectDetectionArray[i + 8], checkedObjectDetectionArray[i + 9]);
      Translation2d corner4 =
          new Translation2d(
              checkedObjectDetectionArray[i + 10], checkedObjectDetectionArray[i + 11]);

      objectDetectionObservations.add(
          new ObjectDetection(
              gamePiece,
              horizontalAngle,
              verticalAngle,
              targetArea,
              corner1,
              corner2,
              corner3,
              corner4));
    }
  }

  private GamePiece convertIDToGamePiece(int id) {
    return switch (id) {
      case 0 -> GamePiece.ALAGE;
      case 1 -> GamePiece.CORAL;
      default -> GamePiece.NONE;
    };
  }
}
