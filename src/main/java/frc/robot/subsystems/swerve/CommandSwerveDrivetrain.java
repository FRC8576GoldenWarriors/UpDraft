package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class CommandSwerveDrivetrain {
  SwerveDrivetrainConstants driveTrainConstants;
  SwerveModuleConstants<?, ?, ?>[] moduleConstants;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    this.driveTrainConstants = driveTrainConstants;
    // Regulate module constants if in simulation mode
    this.moduleConstants = modules;
  }

  public SwerveDrivetrainConstants getDriveTrainConstants() {
    return driveTrainConstants;
  }

  public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
    return moduleConstants;
  }
}
