package frc.robot.subsystems.vision.object;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.AutoLog;

public interface VisionObjectIO extends VisionIO {

  @AutoLog
  class VisionObjectIOInputs extends VisionIO.VisionIOInputs {
    public String objectClass = "";
    public Detection[] objectDetections = new Detection[0];
  }

  public record Detection(
      int id,
      Angle horizontalAngle,
      Angle verticalAngle,
      Dimensionless targetArea,
      double corner0X,
      double corner0Y,
      double corner1X,
      double corner1Y,
      double corner2X,
      double corner2Y,
      double corner3X,
      double corner3Y) {}
}
