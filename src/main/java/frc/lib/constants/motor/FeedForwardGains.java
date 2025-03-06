package frc.lib.constants.motor;

public class FeedForwardGains {
    public final double kS;
    public final double kG;
    public final double kV;
    public final double kA;

    public FeedForwardGains(double kS, double kG, double kV, double kA) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }
}
