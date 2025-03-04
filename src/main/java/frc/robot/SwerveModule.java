// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

public class SwerveModule {

    // TODO: Remove these constants:
    ////////////////////////////////////////////
    private static final double sparkFlexMaxSpeed = 3.6576;
    ////////////////////////////////////////////
    private String moduleName;
    public SwerveModuleState desiredState;

    private final SparkFlex m_driveMotor;
    private final SparkFlex m_turningMotor;

    private final RelativeEncoder rel_driveEncoder;
    private final RelativeEncoder rel_angleEncoder;

    private final CANcoder m_turningEncoder;
    private final RevSwerveModuleConstants constants;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel   PWM output for the drive motor.
     * @param turningMotorChannel PWM output for the turning motor.
     * @param turningEncoderCANID CAN ID associated with CTR-E CANCoder
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderCANID,
            RevSwerveModuleConstants constants,
            String name,
            Boolean invert) {

        this.constants = constants; // NOTE: is this right?? idk
        this.moduleName = name;
        m_driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new SparkFlex(turningMotorChannel, MotorType.kBrushless);

        SparkFlexConfig config_m_drivingMotor = new SparkFlexConfig();
        config_m_drivingMotor.encoder.positionConversionFactor(Constants.Swerve.driveRevToMeters);
        config_m_drivingMotor.encoder.velocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);
        config_m_drivingMotor.closedLoop.p(Constants.Swerve.driveKP);
        config_m_drivingMotor.closedLoop.i(Constants.Swerve.driveKI);
        config_m_drivingMotor.closedLoop.d(Constants.Swerve.driveKD);
        config_m_drivingMotor.closedLoop.velocityFF(1 / 565); // Kv=565 for NEO Vortex (look up on website)
        config_m_drivingMotor.inverted(invert);

        SparkFlexConfig config_m_turningMotor = new SparkFlexConfig();
        config_m_turningMotor.encoder.positionConversionFactor(Constants.Swerve.DegreesPerTurnRotation);
        config_m_turningMotor.encoder.velocityConversionFactor(Constants.Swerve.DegreesPerTurnRotation / 60);
        config_m_turningMotor.closedLoop.p(Constants.Swerve.angleKP);
        config_m_turningMotor.closedLoop.i(Constants.Swerve.angleKI);
        config_m_turningMotor.closedLoop.d(Constants.Swerve.angleKD);
        config_m_turningMotor.closedLoop.outputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        config_m_turningMotor.closedLoopRampRate(Constants.Swerve.angleRampRate);
        config_m_turningMotor.smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);

        // Encoders inside the motor are configured and initial positions are set
        m_driveMotor.configure(config_m_drivingMotor, SparkBase.ResetMode.kResetSafeParameters, null);
        m_turningMotor.configure(config_m_turningMotor, SparkBase.ResetMode.kResetSafeParameters, null);
        rel_driveEncoder = m_driveMotor.getEncoder();
        rel_driveEncoder.setPosition(0);
        rel_angleEncoder = m_turningMotor.getEncoder();

        m_turningEncoder = new CANcoder(turningEncoderCANID, "rio");

        synchronizeEncoders();
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
        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if (Math.abs(this.desiredState.speedMetersPerSecond) <= (sparkFlexMaxSpeed * 0.01)) {
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
            double percentOutput = this.desiredState.speedMetersPerSecond / sparkFlexMaxSpeed;
            m_driveMotor.set(percentOutput);
        } else {
            double velocity = this.desiredState.speedMetersPerSecond;
            driving_controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }

        // System.out.println( "(" + device_name + ")" + "degRef: " + degReference + "
        // desired_speed: " + this.desiredState.speedMetersPerSecond + " current
        // cancoder angle: " + currentAngle);
    }

    public void synchronizeEncoders() {
        double current_canCoder_state = getCANcoderAngle().getDegrees();
        double absolutePosition = current_canCoder_state - this.constants.angleOffset.getDegrees();
        System.out.println("(" + moduleName + ")" + " absolute positions: " + absolutePosition + " cancoder position: "
                + current_canCoder_state);
        rel_angleEncoder.setPosition(-absolutePosition);
    }
}
