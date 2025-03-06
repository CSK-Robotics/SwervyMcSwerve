package frc.lib.constants.motor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MotionConstraints {
    public final double kRampRate;
    public final double kPositionTolerance;
    public final double kLowerLimit;
    public final double kUpperLimit;
    public final TrapezoidProfile.Constraints constraints;

    public MotionConstraints(double kRampRate, double kPositionTolerance, TrapezoidProfile.Constraints constraints,
            double kLowerLimit, double kUpperLimit) {
        this.kRampRate = kRampRate;
        this.kPositionTolerance = kPositionTolerance;
        this.kLowerLimit = kLowerLimit;
        this.kUpperLimit = kUpperLimit;
        this.constraints = constraints;
    }
}
