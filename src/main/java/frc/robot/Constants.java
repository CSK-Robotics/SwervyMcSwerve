package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import frc.lib.constants.motor.CurrentLimits;
import frc.lib.constants.motor.FeedForwardGains;
import frc.lib.constants.motor.MotionConstraints;
import frc.lib.constants.motor.MotorConfig;
import frc.lib.constants.motor.SensorConfig;
import frc.lib.constants.motor.SimulationDetails;
import frc.lib.constants.subsystem.ArmConfig;
import frc.lib.constants.subsystem.SubsystemConstants;
import frc.lib.constants.subsystem.WheelConfig;
import frc.lib.swerve.ModuleConstants;
import frc.lib.swerve.SwerveInstances;

public final class Constants {
    public static final double stickDeadband = 0.05;

    public static final class Swerve {
        // Gyro
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        // Swerve Module Type
        public static final SwerveInstances instanceConstants = SwerveInstances
                .SDSMK4i(SwerveInstances.driveGearRatios.SDSMK4i_L2);

        /* Drivebase Dimensions */
        public static final double trackWidth = Units.inchesToMeters(23.75);
        public static final double wheelBase = Units.inchesToMeters(23.75);

        /* Swerve Current Limiting */
        public static final CurrentLimits angleLimits = new CurrentLimits(20, 40, 0.1, true);
        public static final CurrentLimits driveLimits = new CurrentLimits(35, 60, 0.1, true);

        /* Motor PID Values */
        public static final ClosedLoopConfig anglePID = new ClosedLoopConfig().pidf(0.05, 0.0, 0.0, 0.0,
                ClosedLoopSlot.kSlot0);
        public static final ClosedLoopConfig drivePID = new ClosedLoopConfig().pidf(0.04, 0.0, 0.0,
                1 / 565 /* Kv=565 for NEO Vortex (look up on website) */, ClosedLoopSlot.kSlot0);

        /** Drivetrain Constraints */
        public static final double timeToMaxLinearVelocity = 3.6576; // seconds
        public static final double timeToMaxAngularVelocity = 5.0; // seconds

        /* Module Specific Constants */
        public static class Modules {
            /* Front Left Module - Module 0 */
            public static final ModuleConstants mod0Constants = new ModuleConstants(2, 3, 10,
                    Rotation2d.fromDegrees(0));
            /* Front Right Module - Module 1 */
            public static final ModuleConstants mod1Constants = new ModuleConstants(4, 5, 11,
                    Rotation2d.fromDegrees(0));
            /* Back Left Module - Module 2 */
            public static final ModuleConstants mod2Constants = new ModuleConstants(6, 7, 12,
                    Rotation2d.fromDegrees(0));
            /* Back Right Module - Module 3 */
            public static final ModuleConstants mod3Constants = new ModuleConstants(8, 9, 13,
                    Rotation2d.fromDegrees(0));
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

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    public static RobotConfig pathplannerConfig;
    public class AutonomousData {
        public int desiredAprilTag;
        public int pipelineID;
        public String starterPathPlanner;
        public double areaAprilTag;
        public double direction;

        public AutonomousData(int desiredAprilTag, int pipelineID, String starterPathPlanner, double areaAprilTag, double direction) {
            this.desiredAprilTag = desiredAprilTag;
            this.pipelineID = pipelineID;
            this.starterPathPlanner = starterPathPlanner;
            this.areaAprilTag = areaAprilTag;
            this.direction = direction;
        }
    }

    public AutonomousData LeftPosition = new AutonomousData(1,1,"",500.0,1.0);
    public AutonomousData RightPosition = new AutonomousData(2,2,"",500.0,1.0);
    public AutonomousData TopPosition = new AutonomousData(3,3,"",500.0,1.0);

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

    public static class ElevatorConstants {
        public static final double kElevatorHeight = 0.0;
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

    // TODO: #15 Find the actual values for Coral
    public static final class CoralConstants extends SubsystemConstants {
        private static final int kArmMotorID = 0;
        private static final int kWheelMotorID = 0;
        private static final int kLaserCANID = 0;
        private static final double kArmGearboxRatio = 0.0; // Example value, replace with actual value
        private static final double kArmMOI = 0.0; // Example value, replace with actual value
        private static final double kArmExternalRatio = 1.0; // Example value, replace with actual value

        private static final double kWheelGearboxRatio = 0.0; // Example value, replace with actual value
        private static final double kWheelMOI = 0.0; // Example value, replace with actual value
        private static final double kWheelExternalRatio = 1.0; // Example value, replace with actual value
        private static final double[] kStandardDevs = { 0.0, 0.1 }; // Example values, replace with actual
                                                                    // values
        private static final double kArmLength = 0.0; // Example value, replace with actual value
        private static final FeedForwardGains kArmFeedForwardGains = new FeedForwardGains(0, 0, 0, 0); // Example
                                                                                                       // values,
                                                                                                       // replace
                                                                                                       // with
                                                                                                       // actual
                                                                                                       // values
        private static final double kMaxVelocity = 0;
        private static final double kMaxAcceleration = 0;
        private static final CurrentLimits kWheelLimits = new CurrentLimits(0, 0, 0.0, true);
        private static final CurrentLimits kArmLimits = new CurrentLimits(0, 0, 0.0, true);
        private static final double kArmRampRate = 0;
        private static final double kArmPositionTolerance = 0;
        private static final double kArmUpperLimit = 0;
        private static final double kArmLowerLimit = 0;
        private static final ClosedLoopConfig kArmGains = new ClosedLoopConfig()
                .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
        private static final RegionOfInterest kROI = new RegionOfInterest(8, 8, 16, 16);
        private static final double kCoralDetectTolerance = 0;
        private static final double kCoralDetectDistance = 0;

        private CoralConstants() {
            super(new ArmConfig(
                    new MotorConfig(kArmMotorID, kArmLimits, kArmFeedForwardGains,
                            new MotionConstraints(kArmRampRate, kArmPositionTolerance,
                                    new TrapezoidProfile.Constraints(kMaxVelocity,
                                            kMaxAcceleration),
                                    kArmLowerLimit, kArmUpperLimit),
                            kArmGains,
                            new SimulationDetails(kArmGearboxRatio, kArmExternalRatio,
                                    kArmMOI, kStandardDevs)),
                    kArmLength),
                    new WheelConfig(
                            new MotorConfig(kWheelMotorID, kWheelLimits, null, null, null,
                                    new SimulationDetails(kWheelGearboxRatio,
                                            kWheelExternalRatio, kWheelMOI,
                                            kStandardDevs)),
                            new SensorConfig(kLaserCANID, kCoralDetectDistance,
                                    kCoralDetectTolerance, kROI)));
        }

        public static final CoralConstants instance = new CoralConstants();
    }

    // TODO: #16 Find the actual values for Coral
    public static final class AlgaeConstants extends SubsystemConstants {
        private static final int kArmMotorID = 0;
        private static final int kWheelMotorID = 0;
        private static final int kLaserCANID = 0;
        private static final double kArmGearboxRatio = 0.0; // Example value, replace with actual value
        private static final double kArmMOI = 0.0; // Example value, replace with actual value
        private static final double kArmExternalRatio = 1.0; // Example value, replace with actual value

        private static final double kWheelGearboxRatio = 0.0; // Example value, replace with actual value
        private static final double kWheelMOI = 0.0; // Example value, replace with actual value
        private static final double kWheelExternalRatio = 1.0; // Example value, replace with actual value
        private static final double[] kStandardDevs = { 0.0, 0.1 }; // Example values, replace with actual
                                                                    // values
        private static final double kArmLength = 0.0; // Example value, replace with actual value
        private static final FeedForwardGains kArmFeedForwardGains = new FeedForwardGains(0, 0, 0, 0); // Example
                                                                                                       // values,
                                                                                                       // replace
                                                                                                       // with
                                                                                                       // actual
                                                                                                       // values
        private static final double kMaxVelocity = 0;
        private static final double kMaxAcceleration = 0;
        private static final CurrentLimits kWheelLimits = new CurrentLimits(0, 0, 0.0, true);
        private static final CurrentLimits kArmLimits = new CurrentLimits(0, 0, 0.0, true);
        private static final double kArmRampRate = 0;
        private static final double kArmPositionTolerance = 0;
        private static final double kArmUpperLimit = 0;
        private static final double kArmLowerLimit = 0;
        private static final ClosedLoopConfig kArmGains = new ClosedLoopConfig()
                .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
        private static final RegionOfInterest kROI = new RegionOfInterest(8, 8, 16, 16);
        private static final double kAlgaeDetectTolerance = 0;
        private static final double kAlgaeDetectDistance = 0;

        private AlgaeConstants() {
            super(new ArmConfig(
                    new MotorConfig(kArmMotorID, kArmLimits, kArmFeedForwardGains,
                            new MotionConstraints(kArmRampRate, kArmPositionTolerance,
                                    new TrapezoidProfile.Constraints(kMaxVelocity,
                                            kMaxAcceleration),
                                    kArmLowerLimit, kArmUpperLimit),
                            kArmGains,
                            new SimulationDetails(kArmGearboxRatio, kArmExternalRatio,
                                    kArmMOI, kStandardDevs)),
                    kArmLength),
                    new WheelConfig(
                            new MotorConfig(kWheelMotorID, kWheelLimits, null, null, null,
                                    new SimulationDetails(kWheelGearboxRatio,
                                            kWheelExternalRatio, kWheelMOI,
                                            kStandardDevs)),
                            new SensorConfig(kLaserCANID, kAlgaeDetectDistance,
                                    kAlgaeDetectTolerance, kROI)));
        }

        public static final AlgaeConstants instance = new AlgaeConstants();
    }
}
