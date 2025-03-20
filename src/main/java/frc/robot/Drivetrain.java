// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.studica.frc.*;
import com.studica.frc.AHRS.NavXComType;;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.Swerve.wheelBase / 2.0, -Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.Swerve.wheelBase / 2.0, -Constants.Swerve.trackWidth / 2.0);

  private final SwerveModule m_frontLeft = new SwerveModule(8, 7, 9, Constants.Swerve.Modules.Mod0.constants, "m_frontLeft", false);
  private final SwerveModule m_frontRight = new SwerveModule(2, 1, 10, Constants.Swerve.Modules.Mod1.constants, "m_frontRight", true);
  private final SwerveModule m_backLeft = new SwerveModule(4, 3, 11, Constants.Swerve.Modules.Mod2.constants, "m_backLeft", false);
  private final SwerveModule m_backRight = new SwerveModule(6, 5, 12, Constants.Swerve.Modules.Mod3.constants, "m_backRight", true);

  //private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final AHRS m_gyro = new AHRS(NavXComType.kUSB1); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, 
          m_frontRightLocation,
          m_backLeftLocation,
          m_backRightLocation
        );

  //private final SwerveDrivePoseEstimator m_odometry; 

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  public void setupAutonomousConfigure() {
    System.out.println("Configuring autobuilder...");
    try{
      Constants.pathplannerConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            Constants.pathplannerConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
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

    ChassisSpeeds desiredChassisSpeeds =
      fieldRelative ?
      ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed,
              ySpeed,
              rot,
              m_gyro.getRotation2d()
      )
      : new ChassisSpeeds(
              xSpeed,
              ySpeed,
              rot
      );
    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    
    for (SwerveModuleState swerveModuleState : swerveModuleStates) {
      swerveModuleState.angle = swerveModuleState.angle.times(-1);
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0], "frontLeft", true);
    m_frontRight.setDesiredState(swerveModuleStates[1], "frontRight", true);
    m_backLeft.setDesiredState(swerveModuleStates[2], "backLeft", true);
    m_backRight.setDesiredState(swerveModuleStates[3], "backRight", true);
    
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public LimelightHelpers.PoseEstimate getLimelightPose() {
    // First, tell Limelight your robot's current orientation
    double robotYaw = m_gyro.getAngle();  
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    return limelightMeasurement;
  }

  public void resetPose(Pose2d consumerPose) {
    m_odometry.resetPose(consumerPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return m_kinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                           m_frontRight.getState(),
                                                           m_backLeft.getState(),
                                                           m_backRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false, 1.0);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    var posData_m_frontLeft = m_frontLeft.getPosition();
    var posData_m_frontRight = m_frontRight.getPosition();
    var posData_m_backLeft = m_backLeft.getPosition();
    var posData_m_backRight = m_backRight.getPosition();

    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          posData_m_frontLeft,
          posData_m_frontRight,
          posData_m_backLeft,
          posData_m_backRight
        });
  }

  public void reset() {
    m_gyro.zeroYaw();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", m_gyro.getRotation2d().getDegrees());
  }
}
