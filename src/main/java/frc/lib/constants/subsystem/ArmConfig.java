package frc.lib.constants.subsystem;

import frc.lib.constants.motor.MotorConfig;

public class ArmConfig {
    public final MotorConfig kMotorConfig;
    public final double kArmLength;

    public ArmConfig(MotorConfig kMotorConfig, double kArmLength) {
        this.kMotorConfig = kMotorConfig;
        this.kArmLength = kArmLength;
    }
}
