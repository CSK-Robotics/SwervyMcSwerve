// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  /*
  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);
  */

  private final SwerveModule m_frontLeft = new SwerveModule(8, 7, 9);
  private final SwerveModule m_frontRight = new SwerveModule(2, 1, 10);
  //private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
  //private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation,
          m_frontRightLocation
          /*
          m_backLeftLocation,
          m_backRightLocation
          */
        );

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            /*
            m_backLeft.getPosition(),
            m_backRight.getPosition()
            */
          });

  public Drivetrain() {
    m_gyro.reset();
    m_frontLeft.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(0)), "frontLeft", false);
    m_frontRight.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(0)), "frontRight", false);

  }

  public void wheelsIn() {
    m_frontLeft.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), "frontLeft", false);
    m_frontRight.setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), "frontRight", false);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    //System.out.println(">>>>>> SwerveModuleState (frontLeft): " + swerveModuleStates[0] +"\r\n");
    //System.out.println(">>>>>> SwerveModuleState (frontRight): " + swerveModuleStates[1] +"\r\n");

    m_frontLeft.setDesiredState(swerveModuleStates[0], "frontLeft", true);
    m_frontRight.setDesiredState(swerveModuleStates[1], "frontRight", true);
    /*
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    */
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    var posData_m_frontLeft = m_frontLeft.getPosition();
    var posData_m_frontRight = m_frontRight.getPosition();

    //System.out.println(">>>>>> CANCoder Position (frontLeft): " + posData_m_frontLeft.toString() + "\r\n");
    //System.out.println(">>>>>> CANCoder Position (frontRight): " + posData_m_frontRight.toString() + "\r\n");
    
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          posData_m_frontLeft,
          posData_m_frontRight
          //m_frontLeft.getPosition(),
          /*
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
          */
        });
  }
}
