package frc.lib.configurations.motor;

import java.util.Map;

import com.revrobotics.spark.config.ClosedLoopConfig;

public class MotorConfig {
    public enum ControllerType {
        SPARK_MAX, SPARK_FLEX
    }

    public enum MotorType {
        NEO, NEO550, NEOVORTEX, BAG, MINICIM, CIM, PRO775
    }

    public final MotorType kMotorType;
    public final ControllerType kControllerType;
    public final Map<Integer, Boolean> motorIDs;
    public final FeedForwardGains kFeedForwardGains;
    public final CurrentLimits kCurrentLimits;
    public final MotionConstraints kConstraints;
    public final ClosedLoopConfig kGains;
    public final SimulationDetails kSimulation;

    public MotorConfig(MotorType motorType, ControllerType controllerType, Map<Integer, Boolean> motorIDs,
            CurrentLimits limits, FeedForwardGains feedForwardGains,
            MotionConstraints constraints,
            ClosedLoopConfig gains, SimulationDetails simulation) {
        this.kMotorType = motorType;
        this.kControllerType = controllerType;
        this.motorIDs = motorIDs;
        this.kFeedForwardGains = feedForwardGains;
        this.kCurrentLimits = limits;
        this.kConstraints = constraints;
        this.kGains = gains;
        this.kSimulation = simulation;
    }
}