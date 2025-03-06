package frc.lib.constants.subsystem;

import frc.lib.constants.motor.MotorConfig;
import frc.lib.constants.motor.SensorConfig;

public class WheelConfig {
    public final MotorConfig kMotorConfig;
    public final SensorConfig kSensorConfig;

    public WheelConfig(MotorConfig kMotorConfig, SensorConfig kSensorConfig) {
        this.kMotorConfig = kMotorConfig;
        this.kSensorConfig = kSensorConfig;
    }
}
