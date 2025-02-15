// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import frc.lib.math.GeometryUtils;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  /*
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  */

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.Swerve.wheelBase / 2.0, -Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.Swerve.wheelBase / 2.0, -Constants.Swerve.trackWidth / 2.0);

  public final Auto m_auto_manager = new Auto();

  private final SwerveModule m_frontLeft = new SwerveModule(8, 7, 9, Constants.Swerve.Modules.Mod0.constants, "m_frontLeft", false);
  private final SwerveModule m_frontRight = new SwerveModule(2, 1, 10, Constants.Swerve.Modules.Mod1.constants, "m_frontRight", false);
  private final SwerveModule m_backLeft = new SwerveModule(4, 3, 11, Constants.Swerve.Modules.Mod2.constants, "m_backLeft", false);
  private final SwerveModule m_backRight = new SwerveModule(5, 6, 12, Constants.Swerve.Modules.Mod3.constants, "m_backRight", false);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation,
          m_frontRightLocation,
          m_backLeftLocation,
          m_backRightLocation
        );

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
    //m_frontLeft.synchronizeEncoders();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    // TODO: Convert to for-loop; have kids do this
    positions[0] = m_frontLeft.getPosition();
    positions[1] = m_frontRight.getPosition();
    positions[2] = m_backLeft.getPosition();
    positions[3] = m_backRight.getPosition();
    return positions;
  }

  public Pose2d getPose() {
    return swerveOdometer.getEstimatedPosition();
  }

  public Pose2d resetPose() {
    return 0;
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360).minus(m_gyro.getRotation2d()) : m_gyro.getRotation2d();
  }

  public double getPitch() {
      return m_gyro.getRoll();
  }
    
  public void execute_auto() {
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
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
            config, // The robot configuration
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
    //System.out.println("desired chassis: " + desiredChassisSpeeds);
    //SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0], "frontLeft", true);
    m_frontRight.setDesiredState(swerveModuleStates[1], "frontRight", true);
    m_backLeft.setDesiredState(swerveModuleStates[2], "backLeft", true);
    m_backRight.setDesiredState(swerveModuleStates[3], "backRight", true);
  }

  public void realignWheels() throws InterruptedException {
    m_frontLeft.synchronizeEncoders();
    m_frontRight.synchronizeEncoders();
    m_backLeft.synchronizeEncoders();
    m_backRight.synchronizeEncoders();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    var posData_m_frontLeft = m_frontLeft.getPosition();
    var posData_m_frontRight = m_frontRight.getPosition();
    var posData_m_backLeft = m_backLeft.getPosition();
    var posData_m_backRight = m_backRight.getPosition();

    //System.out.println(">>>>>> CANCoder Position (frontLeft): " + posData_m_frontLeft.toString() + "\r\n");
    //System.out.println(">>>>>> CANCoder Position (frontRight): " + posData_m_frontRight.toString() + "\r\n");
    
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          posData_m_frontLeft,
          posData_m_frontRight,
          posData_m_backLeft,
          posData_m_backRight
        });
  }
}
