package frc.lib.configurations.subsystem;

import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.sensors.SensorConfig;

public class SubsystemConfig {
    public final MotorConfig kMotorConfig;
    public final SensorConfig kSensorConfig;

    public SubsystemConfig(MotorConfig config) {
        this(config, null);
    }

    public SubsystemConfig(MotorConfig motorConfig, SensorConfig sensorConfig) {
        this.kMotorConfig = motorConfig;
        this.kSensorConfig = sensorConfig;
    }
}
