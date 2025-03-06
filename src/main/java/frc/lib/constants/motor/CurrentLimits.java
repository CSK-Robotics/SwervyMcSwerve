package frc.lib.constants.motor;

public class CurrentLimits {
    /* Current Limiting */
    public final int continuousCurrentLimit;
    public final int peakCurrentLimit;
    public final double peakCurrentDuration;
    public final boolean enableCurrentLimit;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param continuousCurrentLimit
     * @param peakCurrentLimit
     * @param peakCurrentDuration
     * @param enableCurrentLimit
     */
    public CurrentLimits(int continuousCurrentLimit, int peakCurrentLimit, double peakCurrentDuration,
            boolean enableCurrentLimit) {
        this.continuousCurrentLimit = continuousCurrentLimit;
        this.peakCurrentDuration = peakCurrentDuration;
        this.peakCurrentLimit = peakCurrentLimit;
        this.enableCurrentLimit = enableCurrentLimit;
    }
}
