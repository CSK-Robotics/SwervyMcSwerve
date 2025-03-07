package frc.lib.configurations.subsystem;

import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SensorConfig;

public class FlywheelConfig extends SubsystemConfig {

    public FlywheelConfig(MotorConfig kMotorConfig, SensorConfig kSensorConfig) {
        super(Type.FLYWHEEL, kMotorConfig, kSensorConfig);
    }
}
