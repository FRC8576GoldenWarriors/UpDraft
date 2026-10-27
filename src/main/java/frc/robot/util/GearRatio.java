package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.ArrayList;
import java.util.List;

public class GearRatio {

  private List<Double> drivingGears;
  private List<Double> drivenGears;

  public GearRatio() {
    drivingGears = new ArrayList<>();
    drivenGears = new ArrayList<>();
  }

  public GearRatio withStage(double driving, double driven) {
    drivingGears.add(driving);
    drivenGears.add(driven);
    return this;
  }

  public Angle motorToMechanism(Angle motorRotation) {
    return motorRotation.div(getRatio());
  }

  public Angle mechanismToMotor(Angle mechanismRotation) {
    return mechanismRotation.times(getRatio());
  }

  public AngularVelocity motorToMechanism(AngularVelocity motorRotation) {
    return motorRotation.div(getRatio());
  }

  public AngularVelocity mechanismToMotor(AngularVelocity mechanismRotation) {
    return mechanismRotation.times(getRatio());
  }

  public Pair<Double, Double> geStage(int stageIndex) {
    if (stageIndex >= drivenGears.size()) {
      return Pair.of(Double.valueOf(1), Double.valueOf(1));
    }

    return Pair.of(
        Double.valueOf(drivingGears.get(stageIndex)), Double.valueOf(drivenGears.get(stageIndex)));
  }

  public double getRatio() {
    double ratio = 1;
    for (int i = 0; i < drivenGears.size(); i++) {
      ratio *= (drivenGears.get(i) / drivingGears.get(i));
    }
    return ratio;
  }
}
