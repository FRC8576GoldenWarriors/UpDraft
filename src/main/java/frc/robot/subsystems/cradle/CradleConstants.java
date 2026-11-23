package frc.robot.subsystems.cradle;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;

public class CradleConstants {
  public static final String LOG_PATH = "Cradle/";

  public static final int FRONT_CANRANGE_ID = 30;
  public static final int BACK_CANRANGE_ID = 31;

  public static final Distance CANRANGE_PROXIMITY_THRESHOLD = Meters.of(0.05);

  public static final Frequency STATUS_SIGNAL_UPDATE_FREQUENCY = Hertz.of(50);

  public static final ProximityParamsConfigs FRONT_CANRANGE_PROXIMITY_PARAMS_CONFIG =
      new ProximityParamsConfigs().withProximityThreshold(CANRANGE_PROXIMITY_THRESHOLD);

  public static final ProximityParamsConfigs BACK_CANRANGE_PROXIMITY_PARAMS_CONFIG =
      new ProximityParamsConfigs().withProximityThreshold(CANRANGE_PROXIMITY_THRESHOLD);
}
