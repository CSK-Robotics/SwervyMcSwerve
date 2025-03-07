package frc.lib.configurations.subsystem;

import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SensorConfig;

public class SubsystemConfig {
    public enum Type {
        ELEVETOR, ARM, FLYWHEEL
    }
    public final Type kType;
    public final MotorConfig kMotorConfig;
    public final SensorConfig kSensorConfig;

    public SubsystemConfig(Type type, MotorConfig config) {
        this(type, config, null);
    }

    public SubsystemConfig(Type type, MotorConfig motorConfig, SensorConfig sensorConfig) {
        this.kMotorConfig = motorConfig;
        this.kSensorConfig = sensorConfig;
        this.kType = type;
    }
}
