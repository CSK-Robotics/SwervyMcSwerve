// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;

import static edu.wpi.first.units.Units.Rotation;

// Phoenix 6 imports:
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.lib.util.swerveUtil.CTREModuleState;

public class SwerveModule {
    private static final double kWheelRadius = 0.0508;
    private static final double kGearRatio = 6.75;

    private static final double kDriveWheelDistanceConversionConstant = Math.PI * 2 * kWheelRadius / kGearRatio;
    private static final int kEncoderResolution = 4096;
    private static final double sparkFlexMaxSpeed = 3.6576;

    //private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    //private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
    public SwerveModuleState desiredState;

    private final SparkFlex m_driveMotor;
    private final SparkFlex m_turningMotor;

    private final RelativeEncoder rel_driveEncoder;
    private final RelativeEncoder rel_angleEncoder;

    private final CANcoder m_turningEncoderPE6;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    /*
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            250,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
    */

    // Gains are for example purposes only - must be determined for your own robot!
    //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
            int turningEncoderCANID) {
        m_driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new SparkFlex(turningMotorChannel, MotorType.kBrushless);
        
        /*
        m_driveMotor.configure(
                new SparkFlexConfig().apply(
                        new EncoderConfig().positionConversionFactor(kDriveWheelDistanceConversionConstant)
                                .velocityConversionFactor(kDriveWheelDistanceConversionConstant * 60)),
                null, null);
        */


        SparkFlexConfig config_m_drivingMotor = new SparkFlexConfig();
        config_m_drivingMotor.encoder.positionConversionFactor(Constants.Swerve.driveRevToMeters);
        config_m_drivingMotor.encoder.velocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);
        config_m_drivingMotor.closedLoop.p(Constants.Swerve.driveKP);
        config_m_drivingMotor.closedLoop.i(Constants.Swerve.driveKI);
        config_m_drivingMotor.closedLoop.d(Constants.Swerve.driveKD);

        SparkFlexConfig config_m_turningMotor = new SparkFlexConfig();
        config_m_turningMotor.encoder.positionConversionFactor(360/kGearRatio);
        config_m_turningMotor.encoder.velocityConversionFactor(360/kGearRatio/60);
        config_m_turningMotor.closedLoop.p(Constants.Swerve.angleKP);
        config_m_turningMotor.closedLoop.i(Constants.Swerve.angleKI);
        config_m_turningMotor.closedLoop.d(Constants.Swerve.angleKD);

        m_driveMotor.configure(config_m_drivingMotor, null, null);
        m_turningMotor.configure(config_m_turningMotor, null, null);

        rel_driveEncoder = m_driveMotor.getEncoder();
        rel_driveEncoder.setPosition(0);

        rel_angleEncoder = m_turningMotor.getEncoder();
        m_turningEncoderPE6 = new CANcoder(turningEncoderCANID, "rio");

        double absolutePosition = getCANcoderAngle().getDegrees();
        rel_angleEncoder.setPosition(absolutePosition);

        // NOTE: Unsure if setting this multiple times can cause issues:
        //BaseStatusSignal.setUpdateFrequencyForAll(100, m_turningEncoderPE6.getAbsolutePosition(),
        //       m_turningEncoderPE6.getVelocity());

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // rel_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
        // kEncoderResolution);

        // Set the distance (in this case, angle) in radians per pulse for the turning
        // encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        //StatusSignal<Angle> position_data = m_turningEncoderPE6.getAbsolutePosition();
        //Angle angle_data = position_data.getValue();

        

        return new SwerveModuleState(
                // rel_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance())
                //rel_driveEncoder.getVelocity(), new Rotation2d(angle_data));
                rel_driveEncoder.getVelocity(), Rotation2d.fromDegrees(rel_angleEncoder.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        StatusSignal<Angle> position_data = m_turningEncoderPE6.getAbsolutePosition();
        Angle angle_data = position_data.getValue();
        return new SwerveModulePosition(
                rel_driveEncoder.getPosition(), new Rotation2d(angle_data));
        // rel_driveEncoder.getDistance(), new
        // Rotation2d(m_turningEncoder.getDistance()));
    }

    public Rotation2d getCANcoderAngle() {
        StatusSignal<Angle> position_data = m_turningEncoderPE6.getAbsolutePosition();
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
        // Optimize the reference state to avoid spinning further than 90 degrees
        Rotation2d currentAngle = getCANcoderAngle();
        this.desiredState = CTREModuleState.optimize(desiredState, currentAngle);

        if(Math.abs(this.desiredState.speedMetersPerSecond) <= (sparkFlexMaxSpeed * 0.01))
        {
            m_turningMotor.stopMotor();
        }

        double degReference = this.desiredState.angle.getDegrees()/360;
        SparkClosedLoopController turning_controller = m_turningMotor.getClosedLoopController();
        turning_controller.setReference(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        
        SparkClosedLoopController driving_controller = m_driveMotor.getClosedLoopController();
        if(isOpenLoop)
        {
            double percentOutput = this.desiredState.speedMetersPerSecond / sparkFlexMaxSpeed;
            m_driveMotor.set(percentOutput);
            System.out.println( "(" + device_name + ")" + " per output: " + percentOutput + " desired_speed: " + this.desiredState.speedMetersPerSecond + " current cancoder angle: " + currentAngle);
        }

        double velocity = this.desiredState.speedMetersPerSecond;
        driving_controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
}
