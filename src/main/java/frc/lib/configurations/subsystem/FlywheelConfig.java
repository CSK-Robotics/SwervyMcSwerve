package frc.lib.configurations.subsystem;

import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SensorConfig;

public class FlywheelConfig extends SubsystemConfig {
    public final double kWheelDiameter;

    public FlywheelConfig(MotorConfig kMotorConfig, SensorConfig kSensorConfig, double kDiameter) {
        super(Type.FLYWHEEL, kMotorConfig, kSensorConfig);
        this.kWheelDiameter = kDiameter;
    }
}
