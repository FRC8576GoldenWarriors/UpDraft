package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.loggedimpl.LoggedSubsystem;

abstract class ArmStateMachine extends LoggedSubsystem {

  public ArmStateMachine(String logPath, StatusSignal<Current>[] armCurrentSignals) {
    super(logPath, armCurrentSignals);
  }

  protected abstract void idling();

  protected abstract void pickingUpCoral();

  protected abstract void movingToCoralL1();

  protected abstract void movingToCoralL2();

  protected abstract void movingToCoralL3();

  protected abstract void movingToCoralL4();

  protected abstract void movingToAlageL1();

  protected abstract void movingToAlageL2();

  protected abstract void movingToAlageFloor();

  protected abstract void movingToAlageLolipop();

  protected abstract void movingToBarge();

  protected abstract void homing();

  protected abstract void standingBy();

  protected abstract void positioningManually();

  protected abstract void zeroing();
}
