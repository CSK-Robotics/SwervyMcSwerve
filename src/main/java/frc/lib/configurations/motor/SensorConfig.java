package frc.lib.configurations.motor;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;

public class SensorConfig {
    public enum Type {
        LASER, ABSOLUTE, LIMIT_SWITCH
    }

    // Laser
    public final int laserCANID;
    public final RegionOfInterest kROI;
    public final double kDetectDistance;

    // Absolute
    public final double kForwardLimit;
    public final double kReverseLimit;

    // Limit Switch
    public final boolean kHasForwardLimit;
    public final boolean kHasReverseLimit;

    // General
    public final Type kType;
    public final double kTolerance;

    /*
     * Constructor for laserCAN sensor configuration
     */
    public SensorConfig(int laserCANID, double detectDistance, double detectTolerance, RegionOfInterest roi) {
        this.kType = Type.LASER;
        this.laserCANID = laserCANID;
        this.kDetectDistance = detectDistance;
        this.kTolerance = detectTolerance;
        this.kROI = roi;

        this.kForwardLimit = 0;
        this.kReverseLimit = 0;
        this.kHasForwardLimit = false;
        this.kHasReverseLimit = false;
    }

    /*
     * Constructor for absolute sensor configuration
     */
    public SensorConfig(double forwardLimit, double reverseLimit, double positionTolerance) {
        this.kType = Type.ABSOLUTE;
        this.kForwardLimit = forwardLimit;
        this.kReverseLimit = reverseLimit;
        this.kTolerance = positionTolerance;

        this.laserCANID = 0;
        this.kDetectDistance = 0;
        this.kROI = null;
        this.kHasForwardLimit = false;
        this.kHasReverseLimit = false;
    }

    /*
     * Constructor for limit switch sensor configuration
     */
    public SensorConfig(boolean hasForwardLimit, boolean hasReverseLimit, double positionTolerance) {
        this.kType = Type.LIMIT_SWITCH;
        this.kHasForwardLimit = hasForwardLimit;
        this.kHasReverseLimit = hasReverseLimit;
        this.kTolerance = positionTolerance;

        this.laserCANID = 0;
        this.kDetectDistance = 0;
        this.kROI = null;
        this.kForwardLimit = 0;
        this.kReverseLimit = 0;
    }
}
