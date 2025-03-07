package frc.lib.configurations.motor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MotionConstraints {
    public final double kZeroingSpeed;
    public final double kRampRate;
    public final double kLowerLimit;
    public final double kUpperLimit;
    public final TrapezoidProfile.Constraints constraints;

    public MotionConstraints(double kZeroingSpeed, double kRampRate, TrapezoidProfile.Constraints constraints, double kLowerLimit,
            double kUpperLimit) {
        this.kZeroingSpeed = kZeroingSpeed;
        this.kRampRate = kRampRate;
        this.kLowerLimit = kLowerLimit;
        this.kUpperLimit = kUpperLimit;
        this.constraints = constraints;
    }
}
