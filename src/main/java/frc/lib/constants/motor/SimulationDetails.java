package frc.lib.constants.motor;

public class SimulationDetails {
    public final double kGearboxRatio;
    public final double kExternalRatio;
    public final double kMOI;
    public final double[] kStandardDevs;

    public SimulationDetails(double kGearboxRatio, double externalRatio, double moi,
            double... standardDevs) {
        this.kGearboxRatio = kGearboxRatio;
        this.kExternalRatio = externalRatio;
        this.kMOI = moi;
        this.kStandardDevs = standardDevs;
    }
}
