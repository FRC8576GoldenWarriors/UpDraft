package frc.robot.subsystems.swerve;

import frc.robot.util.LoggedSubsystem;

abstract class SwerveStateMachine extends LoggedSubsystem {

  public SwerveStateMachine(String logPath) {
    super(logPath);
  }

  protected abstract void teleopDriving();

  protected abstract void rotationLocking();

  protected abstract void wheelLockingWithX();

  protected abstract void idling();

  protected abstract void settingModuleRotations();

  protected abstract void taxiing();

  protected abstract void sysId();
}
