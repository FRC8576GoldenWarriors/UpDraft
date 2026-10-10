package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ArmIOTalonFX implements ArmIO {
    
  @Override
  public StatusSignal<Current>[] getCurrentSignals() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurrentSignals'");
  }

  @Override
  public void idle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'idle'");
  }

  @Override
  public void setWantedArmPosition(Angle position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setWantedArmPosition'");
  }

  @Override
  public boolean home(Current activeHomingCurrent, AngularVelocity activeHomingVelocity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'home'");
  }

  @Override
  public boolean nearSetpoint() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'nearSetpoint'");
  }

@Override
public void updateInputs(ArmIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
}
}
