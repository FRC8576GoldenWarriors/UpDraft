package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionAprilTagIOLimelight implements VisionAprilTagIO {

  private final String limelightName;
  private final NetworkTable limelightTable;

  private final IntegerSubscriber hasTargetSubscriber;
  private final DoubleSubscriber horizontalAngleSubscriber;
  private final DoubleSubscriber verticalAngleSubscriber;
  private final DoubleSubscriber visibleTargetAreaSubscriber;
  private final DoubleSubscriber pipelineLatencySubscriber;
  private final DoubleSubscriber captureLatencySubscriber;
  private final DoubleSubscriber isConnectedSubscriber;
  private final DoubleArraySubscriber hardwareDataSubscriber;
  private final DoubleArraySubscriber imuDataSubscriber;
  private final DoubleArraySubscriber rawCornersSubscriber;

  private final DoubleArraySubscriber mt1RobotPoseSubscriber;
  private final DoubleArraySubscriber mt2RobotPoseSubscriber;

  private final DoubleArraySubscriber megaTagStddevsSubscriber;
  private final IntegerSubscriber mainTagIDSubscriber;
  private final DoubleArraySubscriber rawAprilTagsSubscriber;

  private Optional<double[]> hardwareDataArray = Optional.empty();
  private Optional<double[]> imuDataArray = Optional.empty();
  private Optional<double[]> rawCornerArray = Optional.empty();

  private Optional<TimestampedDoubleArray[]> mt1RobotPoseArray = Optional.empty();
  private Optional<TimestampedDoubleArray[]> mt2RobotPoseArray = Optional.empty();
  private Optional<TimestampedDoubleArray[]> megaTagStddevsArray = Optional.empty();
  private Optional<double[]> rawAprilTagsArray = Optional.empty();

  private ArrayList<AprilTag> aprilTagsObservations = new ArrayList<>();
  private ArrayList<PoseObservation> poseObservations = new ArrayList<>();
  private ArrayList<StandardDeviation> standardDeviations = new ArrayList<>();

  public VisionAprilTagIOLimelight(String limelightName) {
    this.limelightName = limelightName;
    this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

    this.hasTargetSubscriber = limelightTable.getIntegerTopic("tv").subscribe(0);
    this.horizontalAngleSubscriber = limelightTable.getDoubleTopic("tx").subscribe(0);
    this.verticalAngleSubscriber = limelightTable.getDoubleTopic("ty").subscribe(0);
    this.visibleTargetAreaSubscriber = limelightTable.getDoubleTopic("ta").subscribe(0);
    this.pipelineLatencySubscriber = limelightTable.getDoubleTopic("tl").subscribe(0);
    this.captureLatencySubscriber = limelightTable.getDoubleTopic("cl").subscribe(0);
    this.isConnectedSubscriber = limelightTable.getDoubleTopic("hb").subscribe(0);
    this.hardwareDataSubscriber =
        limelightTable.getDoubleArrayTopic("hw").subscribe(new double[] {});
    this.imuDataSubscriber = limelightTable.getDoubleArrayTopic("imu").subscribe(new double[] {});
    this.rawCornersSubscriber =
        limelightTable.getDoubleArrayTopic("tcornxy").subscribe(new double[] {});
    this.mt1RobotPoseSubscriber =
        limelightTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    this.mt2RobotPoseSubscriber =
        limelightTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    this.megaTagStddevsSubscriber =
        limelightTable.getDoubleArrayTopic("stddevs").subscribe(new double[] {});
    this.mainTagIDSubscriber = limelightTable.getIntegerTopic("tid").subscribe(0);
    this.rawAprilTagsSubscriber =
        limelightTable.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionAprilTagIOInputs inputs) {
    updateCachedData();
  }

  private void updateCachedData() {
    hardwareDataArray = checkDoubleArrayForData(hardwareDataSubscriber.get());
    imuDataArray = checkDoubleArrayForData(imuDataSubscriber.get());
    rawCornerArray = checkDoubleArrayForData(rawCornersSubscriber.get());

    mt1RobotPoseArray = checkTimestampedDoubleArraArrayForData(mt1RobotPoseSubscriber.readQueue());
    mt2RobotPoseArray = checkTimestampedDoubleArraArrayForData(mt2RobotPoseSubscriber.readQueue());
    megaTagStddevsArray =
        checkTimestampedDoubleArraArrayForData(megaTagStddevsSubscriber.readQueue());
    rawAprilTagsArray = checkDoubleArrayForData(rawAprilTagsSubscriber.get());
  }

  private Optional<double[]> checkDoubleArrayForData(double[] arrayToCheck) {
    return (arrayToCheck == null || arrayToCheck.length < 1)
        ? Optional.empty()
        : Optional.of(arrayToCheck);
  }

  private Optional<TimestampedDoubleArray[]> checkTimestampedDoubleArraArrayForData(
      TimestampedDoubleArray[] arrayToCheck) {
    return (arrayToCheck == null || arrayToCheck.length < 1)
        ? Optional.empty()
        : Optional.of(arrayToCheck);
  }

  private Optional<TimestampedDoubleArray> checkTimestampedDoubleArrayForData(
      TimestampedDoubleArray arrayToCheck) {
    return (arrayToCheck == null || arrayToCheck.value.length < 1)
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

  private double getHearbeat() {
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

  private void updateMegaTag1Observations() {
    if (mt1RobotPoseArray.isEmpty()) return;
    var observations = mt1RobotPoseArray.get();
    for (TimestampedDoubleArray observation : observations) {
      var optionalCheckedObservation = checkTimestampedDoubleArrayForData(observation);
      if (optionalCheckedObservation.isEmpty()) continue;
      var checkedObservation = optionalCheckedObservation.get();

      Pose3d pose3d =
          new Pose3d(
              new Translation3d(
                  Meters.of(checkedObservation.value[0]),
                  Meters.of(checkedObservation.value[1]),
                  Meters.of(checkedObservation.value[2])),
              new Rotation3d(
                  Degrees.of(checkedObservation.value[3]),
                  Degrees.of(checkedObservation.value[4]),
                  Degrees.of(checkedObservation.value[5])));

      Time totalLatency = Milliseconds.of(checkedObservation.value[6]);
      Time serverTimestamp = Microseconds.of(checkedObservation.timestamp);
      int tagCount = (int) checkedObservation.value[7];
      double tagSpan = checkedObservation.value[8];
      Distance averageTagDistanceFromCamera = Meters.of(checkedObservation.value[9]);
      Dimensionless averageTagArea = Percent.of(checkedObservation.value[10]);

      poseObservations.add(
          new PoseObservation(
              ObservationMethod.MT1,
              pose3d,
              serverTimestamp.minus(totalLatency),
              serverTimestamp,
              totalLatency,
              tagCount,
              tagSpan,
              averageTagDistanceFromCamera,
              averageTagArea));
    }
  }

  private void updateMegaTag2Observations() {
    if (mt2RobotPoseArray.isEmpty()) return;
    var observations = mt2RobotPoseArray.get();
    for (TimestampedDoubleArray observation : observations) {
      var optionalCheckedObservation = checkTimestampedDoubleArrayForData(observation);
      if (optionalCheckedObservation.isEmpty()) continue;
      var checkedObservation = optionalCheckedObservation.get();

      Pose3d pose3d =
          new Pose3d(
              new Translation3d(
                  Meters.of(checkedObservation.value[0]),
                  Meters.of(checkedObservation.value[1]),
                  Meters.of(checkedObservation.value[2])),
              new Rotation3d(
                  Degrees.of(checkedObservation.value[3]),
                  Degrees.of(checkedObservation.value[4]),
                  Degrees.of(checkedObservation.value[5])));

      Time totalLatency = Milliseconds.of(checkedObservation.value[6]);
      Time serverTimestamp = Microseconds.of(checkedObservation.timestamp);
      int tagCount = (int) checkedObservation.value[7];
      double tagSpan = checkedObservation.value[8];
      Distance averageTagDistanceFromCamera = Meters.of(checkedObservation.value[9]);
      Dimensionless averageTagArea = Percent.of(checkedObservation.value[10]);

      poseObservations.add(
          new PoseObservation(
              ObservationMethod.MT2,
              pose3d,
              serverTimestamp.minus(totalLatency),
              serverTimestamp,
              totalLatency,
              tagCount,
              tagSpan,
              averageTagDistanceFromCamera,
              averageTagArea));
    }
  }

  private void updateStandardDeviations() {
    if (megaTagStddevsArray.isEmpty()) return;
    var stddevs = megaTagStddevsArray.get();
    for (TimestampedDoubleArray stddev : stddevs) {
      var optionalCheckedStddev = checkTimestampedDoubleArrayForData(stddev);
      if (optionalCheckedStddev.isEmpty()) continue;
      var checkedStddev = optionalCheckedStddev.get();

      standardDeviations.add(
          new StandardDeviation(
              ObservationMethod.MT1,
              Meters.of(checkedStddev.value[0]),
              Meters.of(checkedStddev.value[1]),
              Meters.of(checkedStddev.value[2]),
              Degrees.of(checkedStddev.value[3]),
              Degrees.of(checkedStddev.value[4]),
              Degrees.of(checkedStddev.value[5]),
              Microseconds.of(checkedStddev.timestamp)));
    }
  }
}
