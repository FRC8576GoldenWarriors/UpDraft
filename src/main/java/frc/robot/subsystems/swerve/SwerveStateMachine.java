package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.loggedimpl.LoggedSubsystem;

abstract class SwerveStateMachine extends LoggedSubsystem {

  public SwerveStateMachine(String logPath, StatusSignal<Current>[] swerveCurrentSignals) {
    super(logPath, swerveCurrentSignals);
  }

  protected abstract void teleopDriving();

  protected abstract void rotationLocking();

  protected abstract void wheelLockingWithX();

  protected abstract void idling();

  protected abstract void settingModuleRotations();

  protected abstract void taxiing();

  protected abstract void sysId();
}
