package frc.lib.constants.motor;

import com.revrobotics.spark.config.ClosedLoopConfig;

public class MotorConfig {
    public final int motorID;
    public final FeedForwardGains kFeedForwardGains;
    public final CurrentLimits kCurrentLimits;
    public final MotionConstraints kConstraints;
    public final ClosedLoopConfig kGains;
    public final SimulationDetails kSimulation;

    public MotorConfig(int motorID, CurrentLimits limits, FeedForwardGains feedForwardGains,
            MotionConstraints constraints,
            ClosedLoopConfig gains, SimulationDetails simulation) {
        this.motorID = motorID;
        this.kFeedForwardGains = feedForwardGains;
        this.kCurrentLimits = limits;
        this.kConstraints = constraints;
        this.kGains = gains;
        this.kSimulation = simulation;
    }
}