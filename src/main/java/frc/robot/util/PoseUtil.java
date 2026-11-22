package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class PoseUtil {

  public static final AprilTagFieldLayout fieldMap =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  public static final double bumperThicknessInInches = 3;

  public static Pose2d getAllianceRelativeZeroPose() {

    Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    double robotWidthWithBumpers =
        Math.abs(Meters.of(TunerConstants.FrontLeft.LocationX).in(Inches))
            + bumperThicknessInInches;
    double robotLengthWithBumpers =
        Math.abs(Meters.of(TunerConstants.FrontLeft.LocationY).in(Inches))
            + bumperThicknessInInches;

    Translation2d allianceTranslation =
        new Translation2d(robotWidthWithBumpers, robotLengthWithBumpers);
    Rotation2d allianceRotation = SwerveConstants.BLUE_PERSPECTIVE_ROTATION;

    if (currentAlliance == Alliance.Red) {
      allianceTranslation =
          new Translation2d(
              Meters.of(fieldMap.getFieldLength()).minus(Inches.of(robotWidthWithBumpers)),
              Meters.of(fieldMap.getFieldWidth()).minus(Inches.of(robotLengthWithBumpers)));
      allianceRotation = SwerveConstants.RED_PERSPECTIVE_ROTATION;
    }

    return new Pose2d(allianceTranslation, allianceRotation);
  }
}
