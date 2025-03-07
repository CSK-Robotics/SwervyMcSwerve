package frc.lib.configurations.subsystem;

import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SensorConfig;

public class LinearConfig extends SubsystemConfig {
    public final double kDrumRadius;
    public final double kCarriageGroundOffset;
    public final double kBaseHeight;

    public LinearConfig(MotorConfig kMotorConfig, SensorConfig kSensorConfig, double kDrumRadius,
            double kCarriageGroundOffset, double kBaseHeight) {
        super(Type.ELEVETOR, kMotorConfig, kSensorConfig);
        this.kDrumRadius = kDrumRadius;
        this.kCarriageGroundOffset = kCarriageGroundOffset;
        this.kBaseHeight = kBaseHeight;
    }

}
