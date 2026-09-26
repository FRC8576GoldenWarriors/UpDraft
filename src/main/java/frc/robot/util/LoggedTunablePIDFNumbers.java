package frc.robot.util;

import java.util.List;
import java.util.function.Consumer;

public class LoggedTunablePIDFNumbers implements LoggedTunable {

    private final String dashboardKey;
    private final boolean subsystemTuningMode;
    
    private final LoggedTunableNumber KP;
    private final LoggedTunableNumber KI;
    private final LoggedTunableNumber KD;

    private final LoggedTunableNumber KS;
    private final LoggedTunableNumber KV;
    private final LoggedTunableNumber KG;

    
    public LoggedTunablePIDFNumbers(String dashboardKey, List<Double> initialPIDFValues, List<Consumer<Double>> changeConsumersPIDF, boolean subsystemTuningMode) {
        this.dashboardKey = "/PID/" + dashboardKey;
        this.subsystemTuningMode = subsystemTuningMode;


        KP = new LoggedTunableNumber(dashboardKey + "/KP", subsystemTuningMode);
        KI = new LoggedTunableNumber(dashboardKey + "/KI", subsystemTuningMode);
        KD = new LoggedTunableNumber(dashboardKey + "/KD", subsystemTuningMode);

        KS = new LoggedTunableNumber(dashboardKey + "/KS", subsystemTuningMode);
        KV = new LoggedTunableNumber(dashboardKey + "/KV", subsystemTuningMode);
        KG = new LoggedTunableNumber(dashboardKey + "/KG", subsystemTuningMode);
        
        initDefault(initialPIDFValues);
        initChangeConsumers(changeConsumersPIDF);
    }

    private void initDefault(List<Double> pidfValues) {
        KP.initDefault(pidfValues.get(0));
        KI.initDefault(pidfValues.get(1));
        KD.initDefault(pidfValues.get(2));

        KS.initDefault(pidfValues.get(3));
        KV.initDefault(pidfValues.get(4));
        KG.initDefault(pidfValues.get(5));
    }

    private void initChangeConsumers(List<Consumer<Double>> changeConsumersPIDF) {
        KP.onChange(changeConsumersPIDF.get(0));
        KI.onChange(changeConsumersPIDF.get(1));
        KD.onChange(changeConsumersPIDF.get(2));

        KS.onChange(changeConsumersPIDF.get(3));
        KV.onChange(changeConsumersPIDF.get(4));
        KG.onChange(changeConsumersPIDF.get(5));
    }

    public boolean hasAnyChanged() {
        return KP.hasChanged() || KI.hasChanged() || KD.hasChanged() || KS.hasChanged() || KV.hasChanged() || KG.hasChanged();
    }

    @Override
    public void checkForChange() {
        if(!subsystemTuningMode)
            return;
        KP.checkForChange();
        KI.checkForChange();
        KD.checkForChange();

        KS.checkForChange();
        KV.checkForChange();
        KG.checkForChange();
    }

    public double getKP() {
        return KP.get();
    }

    public double getKI() {
        return KI.get();
    }

    public double getKD() {
        return KD.get();
    }

    public double getKS() {
        return KS.get();
    }

    public double getKV() {
        return KV.get();
    }

    public double getKG() {
        return KG.get();
    }

}
