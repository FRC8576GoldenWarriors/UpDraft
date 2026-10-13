package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;

public interface VisionIO {

  default void updateInputs(VisionIOInputs inputs) {}

  default String getLimelightName() {
    return "";
  }

  class VisionIOInputs {
    public boolean hasTarget = false;
    public Angle horizontalAngle = Degrees.zero();
    public Angle verticalAngle = Degrees.zero();
    public Dimensionless visibleTargetArea = Percent.zero();
    public Time pipelineLatency = Milliseconds.zero();
    public Time captureLatency = Milliseconds.zero();
    public double hearbeat = 0;
    public boolean isConnected = false;

    public Temperature cpuTemperature = Celsius.zero();
    public Dimensionless cpuUsage = Percent.zero();
    public Dimensionless ramUsage = Percent.zero();
    public double framesPerSecond = 0;

    public Angle suppliedRobotYaw = Degrees.zero();

    public Rotation3d limelightRotation3d = Rotation3d.kZero;
    public Rotation3d limelightRotationalVelocity3d = Rotation3d.kZero;

    public Translation3d limelightAcceleration3d = Translation3d.kZero;

    public double[] rawCorners = new double[0];
  }

  default void cropCameraFOV(double minX, double maxX, double minY, double maxY) {}

  default void setThrottle(int throttle) {}

  default void enableRewind(boolean enabledRewind) {}

  default void captureRewind(int durationSeconds) {}

  default void setIMUMode(int mode) {}

  default void setIMUAssist(double assist) {}
}
