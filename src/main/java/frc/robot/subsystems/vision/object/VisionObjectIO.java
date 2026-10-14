package frc.robot.subsystems.vision.object;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.AutoLog;

public interface VisionObjectIO extends VisionIO {

  public enum GamePiece {
    ALAGE,
    CORAL,
    NONE
  }

  default void updateInputs(VisionObjectIOInputs inputs) {};

  @AutoLog
  class VisionObjectIOInputs extends VisionIO.VisionIOInputs {
    public String objectClass = "";
    public ObjectDetection[] objectDetections = new ObjectDetection[0];
  }

  public record ObjectDetection(
      GamePiece gamePiece,
      Angle horizontalAngle,
      Angle verticalAngle,
      Dimensionless targetArea,
      Translation2d corner1,
      Translation2d corner2,
      Translation2d corner3,
      Translation2d corner4) {}
}
