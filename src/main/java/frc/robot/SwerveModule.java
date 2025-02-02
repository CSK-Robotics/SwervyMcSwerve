// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

// Phoenix 6 imports:
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveModule {
  private static final Distance kWheelDiameter = Inch.of(4);
  private static final double kDriveGearRatio = 6.75;
  private static final Distance kDriveConversionConstant = kWheelDiameter.times(2 * Math.PI / kDriveGearRatio);

  private static final double kTurnGearRatio = 21.4285714286;
  private static final Angle kTurnConversionConstant = Degree.of(360 / kTurnGearRatio);

  private static final DCMotor kDriveMotor = DCMotor.getNeoVortex(1).withReduction(kDriveGearRatio);
  private static final DCMotor kTurnMotor = DCMotor.getNeoVortex(1).withReduction(kTurnGearRatio);

  private static final AngularVelocity kModuleMaxAngularVelocity = kTurnConversionConstant
      .times(kTurnMotor.freeSpeedRadPerSec * 180 / Math.PI).per(Second); // Theoretical maximum limit. TODO: update to
                                                                         // maximum physical limits.
  private static final AngularAcceleration kModuleMaxAngularAcceleration = RadiansPerSecondPerSecond.of(2 * Math.PI);

  private static final LinearVelocity kModuleMaxLinearVelocity = kDriveConversionConstant
      .times(kDriveMotor.freeSpeedRadPerSec * 180 / Math.PI).per(Second); // Theoretical maximum limit. TODO: update to
                                                                          // maximum physical limits.
  private static final LinearAcceleration kModuleMaxLinearAcceleration = kModuleMaxLinearVelocity
      .times(0.5).per(Second);

  private final SparkFlex m_driveMotor;
  private final SparkFlex m_turningMotor;
  private final RelativeEncoder m_driveEncoder;
  private final SparkClosedLoopController m_driveController;
  private final SparkClosedLoopController m_turningController;

  private final CANcoder m_azimuthEncoder;
  public final StatusSignal<Angle> m_azimuthPositionSignal;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);

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
    m_driveController = m_driveMotor.getClosedLoopController();
    m_driveEncoder = m_driveMotor.getEncoder(); // RelativeEncoder

    m_turningMotor = new SparkFlex(turningMotorChannel, MotorType.kBrushless);
    m_turningController = m_turningMotor.getClosedLoopController();

    m_driveMotor.configure(new SparkFlexConfig()
        .apply(new EncoderConfig()
            .positionConversionFactor(kDriveConversionConstant.magnitude()) // Rotations to inches
            .velocityConversionFactor(kDriveConversionConstant.magnitude() * 60))// RPM to inches per second
        .apply(new ClosedLoopConfig()
            .pid(0.1, 0, 0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-kDriveMotor.nominalVoltageVolts / 12, kDriveMotor.nominalVoltageVolts / 12)
            .apply(new MAXMotionConfig()
                .maxAcceleration(kModuleMaxLinearAcceleration.magnitude())
                .maxVelocity(kModuleMaxLinearVelocity.magnitude())
                .allowedClosedLoopError(1))),
        null, null);

    m_turningMotor.configure(new SparkFlexConfig()
        .apply(new EncoderConfig()
            .positionConversionFactor(kTurnConversionConstant.magnitude()) // Rotations to degrees
            .velocityConversionFactor(kTurnConversionConstant.magnitude() * 60))// RPM to degrees per second
        .apply(new ClosedLoopConfig()
            .pid(0.1, 0, 0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-kTurnMotor.nominalVoltageVolts / 12, kTurnMotor.nominalVoltageVolts / 12)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-180, 180)
            .apply(new MAXMotionConfig()
                .maxAcceleration(kModuleMaxAngularAcceleration.magnitude())
                .maxVelocity(kModuleMaxAngularVelocity.magnitude())
                .allowedClosedLoopError(1))),
        null, null);

    m_azimuthEncoder = new CANcoder(turningEncoderCANID, "rio");
    m_azimuthPositionSignal = m_azimuthEncoder.getAbsolutePosition();

    // NOTE: Unsure if setting this multiple times can cause issues:
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_azimuthPositionSignal/* , m_azimuthVelocitySignal */);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_azimuthPositionSignal.getValue()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_azimuthPositionSignal.getValue()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_azimuthPositionSignal.getValue());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);
    var error = desiredState.angle.minus(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired direction of travel that can occur when modules change
    // directions. This results in smoother driving.
    desiredState.speedMetersPerSecond *= error.getCos();

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    m_driveController.setReference(desiredState.speedMetersPerSecond,
        ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, driveFeedforward);

    m_turningController.setReference(error.getDegrees(),
        ControlType.kMAXMotionPositionControl);
  }
}
