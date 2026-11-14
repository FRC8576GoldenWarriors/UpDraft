package frc.robot.subsystems.cradle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.generated.TunerConstants;
import frc.robot.util.StatusSignalRefresher;
import frc.robot.util.alertmanager.AlertManager;
import java.util.function.BooleanSupplier;

public class CradleCANrangeIO implements CradleIO {

  private final CANrange frontCANrange;
  private final CANrange backCANrange;

  private final BooleanSupplier frontCANrangeConnectedSignal;
  private final StatusSignal<Boolean> frontCANrangeDetectedSignal;
  private final StatusSignal<Distance> frontCANrangeDistanceSignal;

  private final BooleanSupplier backCANrangeConnectedSignal;
  private final StatusSignal<Boolean> backCANrangeDetectedSignal;
  private final StatusSignal<Distance> backCANrangeDistanceSignal;

  public CradleCANrangeIO() {
    frontCANrange = new CANrange(CradleConstants.FRONT_CANRANGE_ID, TunerConstants.kCANBus);
    backCANrange = new CANrange(CradleConstants.BACK_CANRANGE_ID, TunerConstants.kCANBus);

    frontCANrangeConnectedSignal = () -> frontCANrange.isConnected();
    frontCANrangeDetectedSignal = frontCANrange.getIsDetected();
    frontCANrangeDistanceSignal = frontCANrange.getDistance();

    backCANrangeConnectedSignal = () -> backCANrange.isConnected();
    backCANrangeDetectedSignal = backCANrange.getIsDetected();
    backCANrangeDistanceSignal = backCANrange.getDistance();

    AlertManager.getInstance()
        .registerAlert(
            new Alert("Front CANrange disconnected.", AlertType.kError),
            frontCANrangeConnectedSignal);
    AlertManager.getInstance()
        .registerAlert(
            new Alert("Back CANrange disconnected.", AlertType.kError),
            backCANrangeConnectedSignal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        CradleConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
        frontCANrangeDetectedSignal,
        frontCANrangeDistanceSignal,
        backCANrangeDetectedSignal,
        backCANrangeDistanceSignal);

    StatusSignalRefresher.getInstance()
        .addStatusSignals(
            frontCANrangeDetectedSignal,
            frontCANrangeDistanceSignal,
            backCANrangeDetectedSignal,
            backCANrangeDistanceSignal);
  }

  @Override
  public StatusSignal<Current>[] getCurrentSignals() {
    return new StatusSignal[0];
  }

  @Override
  public void updateInputs(CradleIOInputs inputs) {
    inputs.frontCANrangeConnected = frontCANrangeConnectedSignal.getAsBoolean();
    inputs.frontCANrangeDetected = frontCANrangeDetectedSignal.getValue();
    inputs.frontCANrangeDistance = frontCANrangeDistanceSignal.getValue();

    inputs.backCANrangeConnected = backCANrangeConnectedSignal.getAsBoolean();
    inputs.backCANrangeDetected = backCANrangeDetectedSignal.getValue();
    inputs.backCANrangeDistance = backCANrangeDistanceSignal.getValue();
  }
}
