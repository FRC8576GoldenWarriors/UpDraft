package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.vision.Vision.PoseEstimateConsumer;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.alertmanager.AlertManager;
import frc.robot.util.cameras.CameraConfig;
import frc.robot.util.loggedimpl.LoggedSubsystem;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class ApriltagVision extends LoggedSubsystem {

  private final PoseEstimateConsumer poseEstimateConsumer;
  private final CameraConfig[] cameraConfigs;
  private final List<Pair<VisionAprilTagIOLimelight, VisionAprilTagIOInputsAutoLogged>> ioAndInputs;

  public ApriltagVision(PoseEstimateConsumer poseEstimateConsumer, CameraConfig... cameraConfigs) {
    super(VisionConstants.AprilTagVision.LOG_PATH, null, true);
    this.poseEstimateConsumer = poseEstimateConsumer;
    this.cameraConfigs = cameraConfigs;

    this.ioAndInputs = new ArrayList<>();

    for (CameraConfig cameraConfig : cameraConfigs) {
      var cameraName = cameraConfig.getLimelightName();
      var io = new VisionAprilTagIOLimelight(cameraName);
      var inputs = new VisionAprilTagIOInputsAutoLogged();

      BooleanSupplier alertConnection = () -> inputs.isConnected;

      AlertManager.getInstance()
          .registerAlert(
              Pair.of(
                  new Alert(cameraName + " limelight is disconnected.", AlertType.kError),
                  alertConnection));

      ioAndInputs.add(Pair.of(io, inputs));
    }
  }

  @Override
  public void periodic() {
    for (Pair<VisionAprilTagIOLimelight, VisionAprilTagIOInputsAutoLogged> ioAndInput :
        ioAndInputs) {
      var io = ioAndInput.getFirst();
      var input = ioAndInput.getSecond();

      io.updateInputs(input);
      processInputs(io.getLimelightName(), input);
    }

    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    for (Pair<VisionAprilTagIOLimelight, VisionAprilTagIOInputsAutoLogged> ioAndInput :
        ioAndInputs) {}
  }
}
