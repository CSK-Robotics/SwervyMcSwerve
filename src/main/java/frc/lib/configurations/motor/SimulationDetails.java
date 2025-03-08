package frc.lib.configurations.motor;

public class SimulationDetails {
    public final double kGearboxRatio;
    public final double kExternalRatio;
    public final double kMass;
    public final double[] kStandardDevs;

    public SimulationDetails(double kGearboxRatio, double externalRatio, double mass,
            double... standardDevs) {
        this.kGearboxRatio = kGearboxRatio;
        this.kExternalRatio = externalRatio;
        this.kMass = mass;
        this.kStandardDevs = standardDevs;
    }
}
