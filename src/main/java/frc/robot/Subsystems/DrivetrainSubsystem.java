// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.configurations.subsystem.ModuleConfig;
import frc.lib.subsystems.ISubsystem;
import frc.lib.subsystems.Subsystem.FieldPosition;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase implements ISubsystem {
    // TODO: #18 Add simulation and visualization support to DrivetrainSubsystem
    private final SwerveModule m_frontLeft = new SwerveModule(Swerve.frontLeftConstants);
    private final SwerveModule m_frontRight = new SwerveModule(Swerve.frontRightConstants);
    private final SwerveModule m_backLeft = new SwerveModule(Swerve.backLeftConstants);
    private final SwerveModule m_backRight = new SwerveModule(Swerve.backRightConstants);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
            Swerve.kKinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            }, Swerve.startingPose);

    // field2d
    public final Field2d m_field;

    public DrivetrainSubsystem() {
        m_gyro.reset();

        // field2d
        m_field = new Field2d();
        SmartDashboard.putData(m_field);
    }

    public void setupAutonomousConfigure() {
        System.out.println("Configuring autobuilder...");
        try {
            AutoConstants.pathplannerConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting
                                 // pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the
                                                                      // robot given ROBOT
                                                                      // RELATIVE ChassisSpeeds. Also
                                                                      // optionally outputs
                                                                      // individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following
                                                // controller for
                                                // holonomic drive trains
                        new PIDConstants(AutoConstants.XY_kP, AutoConstants.XY_kI,
                                AutoConstants.XY_kD), // Translation PID constants
                        new PIDConstants(AutoConstants.THETA_kP, AutoConstants.THETA_kI,
                                AutoConstants.THETA_kD) // Rotation PID constants
                ),
                AutoConstants.pathplannerConfig, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
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
    public Command drive(
            double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        System.out.println("x: " + xSpeed + ", y: " + ySpeed + ", theta: " + rot);
        ChassisSpeeds desiredChassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] swerveModuleStates = Swerve.kKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        for (SwerveModuleState swerveModuleState : swerveModuleStates) {
            swerveModuleState.angle = swerveModuleState.angle.times(-1);
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                ModuleConfig.SwerveWheelConfig.kMaxSpeed);

        return m_frontLeft.setDesiredState(swerveModuleStates[0], "frontLeft").alongWith(
                m_frontRight.setDesiredState(swerveModuleStates[1], "frontRight")).alongWith(
                        m_backLeft.setDesiredState(swerveModuleStates[2], "backLeft"))
                .alongWith(
                        m_backRight.setDesiredState(swerveModuleStates[3], "backRight"));
    }

    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
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

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Swerve.kKinematics.toChassisSpeeds(m_frontLeft.getState(),
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
        // LimelightHelpers.PoseEstimate limelightPose = getLimelightPose();
        // m_odometry.addVisionMeasurement(limelightPose.pose,
        // limelightPose.timestampSeconds);
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
        // m_field.setRobotPose(getPose());
    }

    @Override
    public Command aim(FieldPosition goal) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'aim'");
    }

    @Override
    public Command score(FieldPosition goal, BooleanEvent fire) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'score'");
    }

    @Override
    public Command intake(FieldPosition position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'intake'");
    }

    @Override
    public Command zero() {
        return m_frontLeft.zero().alongWith(m_frontRight.zero()).alongWith(m_backLeft.zero())
                .alongWith(m_backRight.zero());
    }

}
