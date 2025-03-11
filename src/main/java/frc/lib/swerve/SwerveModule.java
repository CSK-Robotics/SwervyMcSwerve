// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.configurations.subsystem.AngularConfig;
import frc.lib.configurations.subsystem.LinearConfig;
import frc.lib.configurations.subsystem.ModuleConfig;
import frc.lib.subsystems.AngularSubsystem;
import frc.lib.subsystems.LinearSubsystem;
import frc.lib.subsystems.Subsystem.FieldPosition;

public class SwerveModule {
    // private String moduleName;

    public static final class AngleSubsystem extends AngularSubsystem {
        public static final Map<FieldPosition, FieldPositionValue> POSITIONS;

        static {
            POSITIONS = new EnumMap<>(FieldPosition.class);
            for (FieldPosition pos : FieldPosition.values()) {
                POSITIONS.put(pos, new FieldPositionValue());
            }
        }

        public AngleSubsystem(AngularConfig constants) {
            super(constants, POSITIONS, null);
            POSITIONS.get(FieldPosition.STARTING).accept(0.0);
        }
    }

    private static final class WheelSubsystem extends LinearSubsystem {
        public static final Map<FieldPosition, FieldPositionValue> POSITIONS;

        static {
            POSITIONS = new EnumMap<>(FieldPosition.class);
            for (FieldPosition pos : FieldPosition.values()) {
                POSITIONS.put(pos, new FieldPositionValue());
            }
        }

        public WheelSubsystem(LinearConfig constants) {
            super(constants, POSITIONS, null);
            POSITIONS.get(FieldPosition.STARTING).accept(0.0);
        }
    }

    private final AngleSubsystem m_angle;
    private final WheelSubsystem m_wheel;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel   PWM output for the drive motor.
     * @param turningMotorChannel PWM output for the turning motor.
     * @param turningEncoderCANID CAN ID associated with CTR-E CANCoder
     */
    public SwerveModule(ModuleConfig constants) {
        // moduleName = constants.kName;
        m_wheel = new WheelSubsystem(constants.kWheelConfig);
        m_angle = new AngleSubsystem(constants.kAngularConfig);
    }

    /**
     * Zero the subsystem
     *
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command zero() {
        return m_angle.zero().alongWith(m_wheel.zero());
    }

    // Controls

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public Command setDesiredState(SwerveModuleState desiredState, String device_name) {
        SwerveModuleState optimizedState = ModuleState.optimize(desiredState, getState().angle);

        AngleSubsystem.POSITIONS.get(FieldPosition.DRIVING).accept(optimizedState.angle.getDegrees());
        WheelSubsystem.POSITIONS.get(FieldPosition.DRIVING).accept(optimizedState.speedMetersPerSecond);
        return m_angle.aim(FieldPosition.DRIVING).alongWith(m_wheel.aim(FieldPosition.DRIVING));
    }

    // Telemetry

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_wheel.getVelocity(), Rotation2d.fromDegrees(m_angle.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_wheel.getPosition(), Rotation2d.fromDegrees(m_angle.getPosition()));
    }
}
