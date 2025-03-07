// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DrivetrainSubsystem extends SubsystemBase {
    // TODO: #18 Add simulation and visualization support to DrivetrainSubsystem
    private final SwerveModule m_frontLeft = new SwerveModule(Swerve.Modules.mod0Constants,
            "m_frontLeft");
    private final SwerveModule m_frontRight = new SwerveModule(Swerve.Modules.mod1Constants,
            "m_frontRight");
    private final SwerveModule m_backLeft = new SwerveModule(Swerve.Modules.mod2Constants,
            "m_backLeft");
    private final SwerveModule m_backRight = new SwerveModule(Swerve.Modules.mod3Constants,
            "m_backRight");

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Swerve.wheelBase / 2.0,
                    Swerve.trackWidth / 2.0),
            new Translation2d(Swerve.wheelBase / 2.0,
                    -Swerve.trackWidth / 2.0),
            new Translation2d(-Swerve.wheelBase / 2.0,
                    Swerve.trackWidth / 2.0),
            new Translation2d(-Swerve.wheelBase / 2.0,
                    -Swerve.trackWidth / 2.0));

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    public DrivetrainSubsystem() {
        m_gyro.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     */
    public void drive(
            double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        ChassisSpeeds desiredChassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);

        for (SwerveModuleState swerveModuleState : swerveModuleStates) {
            swerveModuleState.angle = swerveModuleState.angle.times(-1);
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                Swerve.instanceConstants.driveFreeSpeed());

        m_frontLeft.setDesiredState(swerveModuleStates[0], "frontLeft", true);
        m_frontRight.setDesiredState(swerveModuleStates[1], "frontRight", true);
        m_backLeft.setDesiredState(swerveModuleStates[2], "backLeft", true);
        m_backRight.setDesiredState(swerveModuleStates[3], "backRight", true);

    }

    /**
     * Updates the field relative position of the robot.
     */
    private void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

}
