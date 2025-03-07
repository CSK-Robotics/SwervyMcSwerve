package frc.lib.configurations.subsystem;

import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SensorConfig;

public class AngularConfig extends SubsystemConfig {
    public final double kArmLength;

    public AngularConfig(MotorConfig kMotorConfig, SensorConfig kSensorConfig, double kArmLength) {
        super(Type.ARM, kMotorConfig, kSensorConfig);
        this.kArmLength = kArmLength;
    }
}
