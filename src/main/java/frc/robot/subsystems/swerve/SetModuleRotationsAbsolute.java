package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.stream.IntStream;

public class SetModuleRotationsAbsolute implements SwerveRequest {

  public Rotation2d[] moduleRotations = new Rotation2d[4];

  private static final SwerveModule.ModuleRequest moduleRequest = new SwerveModule.ModuleRequest();

  public SetModuleRotationsAbsolute withModuleRotations(final Rotation2d[] moduleRotations) {

    this.moduleRotations = moduleRotations;

    return this;
  }

  @Override
  public StatusCode apply(
      SwerveDrivetrain.SwerveControlParameters parameters,
      SwerveModule<?, ?, ?>... modulesToApply) {
    IntStream.range(0, modulesToApply.length)
        .forEach(
            index -> {
              final SwerveModule<?, ?, ?> module = modulesToApply[index];
              final Rotation2d targetPosition =
                  calculateTargetModulePosition(
                      module.getCurrentState().angle, moduleRotations[index]);
              final SwerveModule.ModuleRequest request =
                  new SwerveModule.ModuleRequest()
                      .withState(new SwerveModuleState(0, targetPosition));

              module.apply(request);
            });
    return StatusCode.OK;
  }

  private static Rotation2d calculateTargetModulePosition(
      Rotation2d currentPosition, final Rotation2d targetPosition) {
    final double currentAngle = currentPosition.getDegrees();
    final double targetAngle = targetPosition.getDegrees();
    final double error = targetAngle - currentAngle;

    if (Math.abs(error) <= 90) {
      return targetPosition;
    }

    final double newTargetAngle = targetAngle - Math.copySign(90, error);

    return Rotation2d.fromDegrees(newTargetAngle);
  }
}
