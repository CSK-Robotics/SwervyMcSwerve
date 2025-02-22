// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/** Represents a swerve drive style drivetrain. */
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

  public void setupAutonomousConfigure() {
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

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d consumerPose) {
        m_odometry.resetPose(consumerPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
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
