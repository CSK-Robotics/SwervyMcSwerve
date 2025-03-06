package frc.lib.constants.motor;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;

public class SensorConfig {
    public final int laserCANID;
    public final RegionOfInterest kROI;
    public final double kDetectDistance;
    public final double kDetectTolerance;

    public SensorConfig(int laserCANID, double detectDistance, double detectTolerance, RegionOfInterest roi) {
        this.laserCANID = laserCANID;
        this.kDetectDistance = detectDistance;
        this.kDetectTolerance = detectTolerance;
        this.kROI = roi;
    }
}
