package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
// import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.lib.util.swerveUtil.SwerveInstances;

public final class Constants {
    public static final double stickDeadband = 0.05;

    public static final class Swerve {
        // Max Output Powers
        public static final double drivePower = 1;
        public static final double anglePower = .9;

        // Gyro
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        // Swerve Module Type
        public static final SwerveInstances chosenModule = SwerveInstances
                .SDSMK4i(SwerveInstances.driveGearRatios.SDSMK4i_L2);
        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        // the number of degrees that a single rotation of the turn motor turns the
        // wheel.
        public static final double DegreesPerTurnRotation = 360 / angleGearRatio;
        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;

        // encoder setup
        // meters per rotation
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double driveRevToMeters = wheelCircumference / (driveGearRatio);
        public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60;
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.75);
        public static final double wheelBase = Units.inchesToMeters(23.75);
        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        /* Angle Motor PID Values */
        public static final double angleKP = 0.05;
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKFF = 0;

        /* Drive Motor info */
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference)
                / driveGearRatio;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.04;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 1 / kDriveWheelFreeSpeedRps;
        /** Meters per Second */
        public static final double maxSpeed = 3.6576;
        /** Radians per Second */
        public static final double maxAngularVelocity = 5.0;
        public static double angleRampRate = 0;

        public static class Modules {
            /* Module Specific Constants */
            /* Front Left Module - Module 0 */
            public static final class Mod0 {

                public static final int driveMotorID = 8;
                public static final int angleMotorID = 7;
                public static final int canCoderID = 9;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// Rotation2d.fromDegrees(37.7);
                public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset);
            }

            /* Front Right Module - Module 1 */
            public static final class Mod1 {
                public static final int driveMotorID = 2;
                public static final int angleMotorID = 1;
                public static final int canCoderID = 10;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
                public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset);
            }

            /* Back Left Module - Module 2 */
            public static final class Mod2 {
                public static final int driveMotorID = 4;
                public static final int angleMotorID = 3;
                public static final int canCoderID = 11;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
                public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset);
            }

            /* Back Right Module - Module 3 */
            public static final class Mod3 {
                public static final int driveMotorID = 6;
                public static final int angleMotorID = 5;
                public static final int canCoderID = 12;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
                public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset);
            }
        }
    }

    public static final class CameraConstants {

        public static final double ROLL = -Math.PI / 2;
        public static final double PITCH = 0.0;
        public static final double YAW = 0.0;
        public static final Transform3d KCAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(8),
                        Units.inchesToMeters(22.125)),
                new Rotation3d(ROLL, PITCH, YAW)).inverse();

        public static final String CAMERA_NAME = "CSI";
        public static final double LARGEST_DISTANCE = 0.1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

        public static final double X_kP = 5;
        public static final double X_kI = 0;
        public static final double X_kD = 0;

        public static final double Y_kP = 5;
        public static final double Y_kI = 0;
        public static final double Y_kD = 0;

        public static final double THETA_kP = 6.2;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0;

        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    public static RobotConfig pathplannerConfig;

    public class AutonomousData {
        public int desiredAprilTag;
        public int pipelineID;
        public String starterPathPlanner;
        public double areaAprilTag;
        public double direction;

        public AutonomousData(int desiredAprilTag, int pipelineID, String starterPathPlanner, double areaAprilTag,
                double direction) {
            this.desiredAprilTag = desiredAprilTag;
            this.pipelineID = pipelineID;
            this.starterPathPlanner = starterPathPlanner;
            this.areaAprilTag = areaAprilTag;
            this.direction = direction;
        }
    }

    public AutonomousData LeftPosition = new AutonomousData(1, 1, "", 500.0, 1.0);
    public AutonomousData RightPosition = new AutonomousData(2, 2, "", 500.0, 1.0);
    public AutonomousData TopPosition = new AutonomousData(3, 3, "", 500.0, 1.0);

    public static final class AutoMotorConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

        public static final double X_kP = 5;
        public static final double X_kI = 0;
        public static final double X_kD = 0;

        public static final double Y_kP = 5;
        public static final double Y_kI = 0;
        public static final double Y_kD = 0;

        public static final double THETA_kP = 6.2;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0;

        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class ElevatorConstants {
        public static final double kElevatorKp = 26.722;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 1.6047;

        public static final double kElevatorkS = 0.01964; // volts (V)
        public static final double kElevatorkV = 3.894; // volt per velocity (V/(m/s))
        public static final double kElevatorkA = 0.173; // volt per acceleration (V/(m/s²))
        public static final double kElevatorkG = 0.91274; // volts (V)

        public static final double kElevatorGearing = 10.0;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kCarriageMass = 4.0; // kg

        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final Distance kLaserCANOffset = Inches.of(3);
        public static final Distance kStartingHeightSim = Meters.of(0);
        public static final Distance kMinElevatorHeight = Meters.of(0.0);
        public static final Distance kMaxElevatorHeight = Meters.of(10.25);

        // TODO: #13 Find the actual values for CarriageGroundOffset, PositionTolerance
        // and kZeroingSpeed
        public static final double kCarriageGroundOffset = 0;
        public static final double kPositionTolerance = 0;
        public static final double kZeroingSpeed = 0;

        public static double kElevatorRampRate = 0.1;
        public static int kElevatorCurrentLimit = 40;
        public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
        public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second)
                .in(MetersPerSecondPerSecond);
    }

    public static class CoralConstants {
        public static final double kArmKp = 26.722;
        public static final double kArmKi = 0;
        public static final double kArmKd = 1.6047;

        public static final double kArmGearing = 25.0;
        // public static final double kArmLength = Units.inchesToMeters(2.0);
        // public static final double kMass = 4.0; // kg

        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final Distance kLaserCANOffset = Inches.of(3);
        // public static final Distance kStartingHeightSim = Meters.of(0);
        // public static final Distance kMinElevatorHeight = Meters.of(0.0);
        public static final Distance kMaxElevatorHeight = Meters.of(10.25);

        // TODO: #13 Find the actual values for CarriageGroundOffset, PositionTolerance
        // and kZeroingSpeed
        // public static final double kCarriageGroundOffset = 0;
        public static final double kPositionTolerance = 0;
        public static final double kZeroingSpeed = 0;

        public static double kArmRampRate = 0.1;
        public static int kArmCurrentLimit = 40;
        public static double kArmMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
        public static double kArmMaxAcceleration = Meters.of(8).per(Second).per(Second)
                .in(MetersPerSecondPerSecond);
    }

    public static class AlgaeConstants {
        public static final double kArmKp = 26.722;
        public static final double kArmKi = 0;
        public static final double kArmKd = 1.6047;

        public static final double kArmGearing = 25.0;
        // public static final double kArmLength = Units.inchesToMeters(2.0);
        // public static final double kMass = 4.0; // kg

        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final Distance kLaserCANOffset = Inches.of(3);
        // public static final Distance kStartingHeightSim = Meters.of(0);
        // public static final Distance kMinElevatorHeight = Meters.of(0.0);
        public static final Distance kMaxElevatorHeight = Meters.of(10.25);

        // TODO: #13 Find the actual values for CarriageGroundOffset, PositionTolerance
        // and kZeroingSpeed
        // public static final double kCarriageGroundOffset = 0;
        public static final double kPositionTolerance = 0;
        public static final double kZeroingSpeed = 0;

        public static double kArmRampRate = 0.1;
        public static int kArmCurrentLimit = 40;
        public static double kArmMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
        public static double kArmMaxAcceleration = Meters.of(8).per(Second).per(Second)
                .in(MetersPerSecondPerSecond);
    }
}
