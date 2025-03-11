package frc.lib.configurations.subsystem;

import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.sensors.SensorConfig;

public class AngularConfig extends SubsystemConfig {
    public enum AngularType {
        VERTICAL, HORIZONTAL
    }

    public final AngularType kAngularType;

    public final double kRadius;

    public AngularConfig(AngularType type, MotorConfig kMotorConfig, SensorConfig kSensorConfig, double kRadius) {
        super(kMotorConfig, kSensorConfig);
        this.kAngularType = type;
        this.kRadius = kRadius;
    }

    public AngularConfig(AngularType type, MotorConfig kMotorConfig, SensorConfig kSensorConfig) {
        super(kMotorConfig, kSensorConfig);
        this.kAngularType = type;
        this.kRadius = 1.0;
    }
}
