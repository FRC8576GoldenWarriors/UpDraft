package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.AutoLog;

public interface VisionAprilTagIO extends VisionIO {

  public enum ObservationMethod {
    MT1,
    MT2,
    NA
  }

  default void updateInputs(VisionAprilTagIOInputs inputs) {}

  @AutoLog
  class VisionAprilTagIOInputs extends VisionIO.VisionIOInputs {
    public AprilTag[] aprilTagsDetected = new AprilTag[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public StandardDeviation[] standardDeviations = new StandardDeviation[0];
  }

  public record AprilTag(
      int id,
      Angle horizontalAngle,
      Angle verticalAngle,
      Dimensionless tagArea,
      Distance distToCamera,
      Distance distToRobot,
      Dimensionless ambiguity) {}

  public record PoseObservation(
      ObservationMethod observationMethod,
      Pose3d megaTagPose3d,
      Time megaTagLatencyCompensatedTimestamp,
      Time megaTagRawTimestamp,
      Time megaTagTotalLatency,
      int megaTagTotalTagCount,
      double megaTagTagSpan,
      Distance megaTagAverageTagDistanceFromCamera,
      Dimensionless megaTagAverageTagArea) {}

  public record StandardDeviation(
      ObservationMethod observationMethod,
      Distance xDev,
      Distance yDev,
      Distance zDev,
      Angle rollDev,
      Angle pitchDev,
      Angle yawDev,
      Time timestamp) {}

  default void setRobotOrientation(Rotation3d robotRotation, Rotation3d robotAngularVelocities) {}
}
