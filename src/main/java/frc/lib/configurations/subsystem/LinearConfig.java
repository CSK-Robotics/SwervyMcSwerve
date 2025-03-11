package frc.lib.configurations.subsystem;

import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.sensors.SensorConfig;

public class LinearConfig extends SubsystemConfig {
    public enum LinearType {
        FLYWHEEL, ELEVETOR, DRIVE, SIMPLE
    }

    public final LinearType kLinearType;

    public final double kRadius;
    public final double kPositionOffset;
    public final double kBaseHeight;

    public LinearConfig(LinearType type, MotorConfig kMotorConfig, SensorConfig kSensorConfig, double kDrumRadius,
            double kCarriageGroundOffset, double kBaseHeight) {
        super(kMotorConfig, kSensorConfig);
        this.kLinearType = type;
        this.kRadius = kDrumRadius;
        this.kPositionOffset = kCarriageGroundOffset;
        this.kBaseHeight = kBaseHeight;
    }

    public LinearConfig(LinearType type, MotorConfig kMotorConfig, SensorConfig kSensorConfig, double kWheelRadius) {
        super(kMotorConfig, kSensorConfig);
        this.kLinearType = type;
        this.kRadius = kWheelRadius;
        this.kPositionOffset = 0.0;
        this.kBaseHeight = 0.0;
    }

}
