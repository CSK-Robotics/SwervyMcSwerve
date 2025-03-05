// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.lib.swerve.ModuleState;
import frc.robot.Constants.Swerve;
import frc.lib.swerve.ModuleConstants;

public class SwerveModule {
    private String moduleName;
    public SwerveModuleState desiredState;

    private final SparkFlex m_driveMotor;
    private final SparkFlex m_turningMotor;

    private final RelativeEncoder rel_driveEncoder;
    private final RelativeEncoder rel_angleEncoder;

    private final CANcoder m_turningEncoder;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel   PWM output for the drive motor.
     * @param turningMotorChannel PWM output for the turning motor.
     * @param turningEncoderCANID CAN ID associated with CTR-E CANCoder
     */
    public SwerveModule(
            ModuleConstants constants,
            String name) {
        this.moduleName = name;
        m_driveMotor = new SparkFlex(constants.driveMotorID, MotorType.kBrushless);
        m_turningMotor = new SparkFlex(constants.angleMotorID, MotorType.kBrushless);

        SparkFlexConfig config_m_drivingMotor = new SparkFlexConfig();
        config_m_drivingMotor.closedLoop.apply(Swerve.drivePID);
        config_m_drivingMotor.encoder
                .positionConversionFactor(Swerve.instanceConstants.drivePositionConversionFactor());
        config_m_drivingMotor.encoder
                .velocityConversionFactor(Swerve.instanceConstants.driveVelocityConversionFactor());
        config_m_drivingMotor.closedLoopRampRate(Swerve.timeToMaxLinearVelocity);
        config_m_drivingMotor.inverted(Swerve.instanceConstants.driveMotorInvert);

        SparkFlexConfig config_m_turningMotor = new SparkFlexConfig();
        config_m_turningMotor.closedLoop.apply(Swerve.anglePID);
        config_m_turningMotor.encoder
                .positionConversionFactor(Swerve.instanceConstants.drivePositionConversionFactor());
        config_m_turningMotor.encoder
                .velocityConversionFactor(Swerve.instanceConstants.driveVelocityConversionFactor());
        config_m_turningMotor.closedLoopRampRate(Swerve.timeToMaxAngularVelocity);
        config_m_turningMotor.smartCurrentLimit(Swerve.angleLimits.continuousCurrentLimit);

        // Encoders inside the motor are configured and initial positions are set
        m_driveMotor.configure(config_m_drivingMotor, SparkBase.ResetMode.kResetSafeParameters, null);
        m_turningMotor.configure(config_m_turningMotor, SparkBase.ResetMode.kResetSafeParameters, null);
        rel_driveEncoder = m_driveMotor.getEncoder();
        rel_driveEncoder.setPosition(0);
        rel_angleEncoder = m_turningMotor.getEncoder();

        m_turningEncoder = new CANcoder(constants.cancoderID, "rio");

        synchronizeEncoders(constants.angleOffset.getDegrees());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                rel_driveEncoder.getVelocity(), Rotation2d.fromDegrees(rel_angleEncoder.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                rel_driveEncoder.getPosition(), Rotation2d.fromDegrees(rel_angleEncoder.getPosition()));
    }

    public Rotation2d getCANcoderAngle() {
        StatusSignal<Angle> position_data = m_turningEncoder.getAbsolutePosition();
        Angle angle_data = position_data.getValue();
        var encoderRotation = new Rotation2d(angle_data);
        return encoderRotation;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, String device_name, boolean isOpenLoop) {
        this.desiredState = ModuleState.optimize(desiredState, getState().angle);

        if (Math.abs(this.desiredState.speedMetersPerSecond) <= (Swerve.instanceConstants.driveFreeSpeed() * 0.01)) {
            m_turningMotor.stopMotor();
        }
        double degReference = this.desiredState.angle.getDegrees();
        // System.out.println("(" + device_name + ")" + "degReference: " +
        // degReference);
        SparkClosedLoopController turning_controller = m_turningMotor.getClosedLoopController();
        turning_controller.setReference(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        // Drive motor controller:
        SparkClosedLoopController driving_controller = m_driveMotor.getClosedLoopController();
        if (isOpenLoop) {
            double percentOutput = this.desiredState.speedMetersPerSecond / Swerve.instanceConstants.driveFreeSpeed();
            m_driveMotor.set(percentOutput);
        } else {
            double velocity = this.desiredState.speedMetersPerSecond;
            driving_controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }

        // System.out.println( "(" + device_name + ")" + "degRef: " + degReference + "
        // desired_speed: " + this.desiredState.speedMetersPerSecond + " current
        // cancoder angle: " + currentAngle);
    }

    public void synchronizeEncoders(double offset) {
        double current_canCoder_state = getCANcoderAngle().getDegrees();
        double absolutePosition = current_canCoder_state - offset;
        System.out.println("(" + moduleName + ")" + " absolute positions: " + absolutePosition + " cancoder position: "
                + current_canCoder_state);
        rel_angleEncoder.setPosition(-absolutePosition);
    }
}
