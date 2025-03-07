package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.Map;

import frc.lib.configurations.motor.CurrentLimits;
import frc.lib.configurations.motor.FeedForwardGains;
import frc.lib.configurations.motor.MotionConstraints;
import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SensorConfig;
import frc.lib.configurations.motor.SimulationDetails;
import frc.lib.configurations.motor.MotorConfig.ControllerType;
import frc.lib.configurations.motor.MotorConfig.MotorType;
import frc.lib.configurations.subsystem.AngularConfig;
import frc.lib.configurations.subsystem.FlywheelConfig;
import frc.lib.configurations.subsystem.LinearConfig;
import frc.lib.swerve.ModuleConstants;
import frc.lib.swerve.SwerveInstances;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Subsystem.FieldPosition;

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
        private static final int kElevatorMotorID = 0;
        private static final int kSecondaryElevatorMotorID = 0;

        private static final ClosedLoopConfig kElevatorGains = new ClosedLoopConfig()
                .pidf(26.722, 0, 1.6047, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
        private static final FeedForwardGains kElevatorFeedForwardGains = new FeedForwardGains(0.01964, 0.91274, 3.894,
                0.173);

        private static final double kElevatorGearing = 10.0;
        private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        private static final double kCarriageMass = 4.0; // kg
        private static final double kExternalRatio = 1.0;
        private static final double[] kStandardDevs = { 0.0, 0.1 };

        // TODO: #13 Find the actual values for CarriageGroundOffset, PositionTolerance
        // and kZeroingSpeed
        private static final double kCarriageGroundOffset = 0;
        private static final double kPositionTolerance = 0;
        private static final double kZeroingSpeed = 0;
        private static final double kTravelDistance = 0;
        private static final double kElevatorHeight = 0.0;

        private static final double kElevatorRampRate = 0.1;
        private static final int kElevatorCurrentLimit = 40;
        private static TrapezoidProfile.Constraints kElevatorConstraints = new TrapezoidProfile.Constraints(4, 8);

        public static final LinearConfig kElevatorConfig = new LinearConfig(
                new MotorConfig(MotorType.NEO, ControllerType.SPARK_MAX,
                        Map.of(kElevatorMotorID, false, kSecondaryElevatorMotorID, true),
                        new CurrentLimits(0, kElevatorCurrentLimit, 0.1, true), kElevatorFeedForwardGains,
                        new MotionConstraints(kZeroingSpeed, kElevatorRampRate, kElevatorConstraints,
                                ElevatorSubsystem.POSITIONS.get(FieldPosition.STARTING), kTravelDistance),
                        kElevatorGains,
                        new SimulationDetails(kElevatorGearing, kExternalRatio, kCarriageMass, kStandardDevs)),
                new SensorConfig(true, true, kPositionTolerance), kElevatorDrumRadius, kCarriageGroundOffset,
                kElevatorHeight);
    }

    // TODO: #15 Find the actual values for Coral
    public static final class CoralConstants {
        private static final int kArmMotorID = 0;
        private static final int kWheelMotorID = 0;
        private static final int kLaserCANID = 0;
        private static final double kArmGearboxRatio = 0.0;
        private static final double kArmMOI = 0.0;
        private static final double kArmExternalRatio = 1.0;
        private static final double kWheelGearboxRatio = 0.0;
        private static final double kWheelMOI = 0.0;
        private static final double kWheelExternalRatio = 1.0;
        private static final double[] kStandardDevs = { 0.0, 0.1 };
        private static final double kArmLength = 0.0;
        private static final FeedForwardGains kArmFeedForwardGains = new FeedForwardGains(0, 0, 0, 0);
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
        public static final double kZeroingSpeed = 0;

        public static final AngularConfig kArmConfig = new AngularConfig(
                new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX,
                        Map.of(kArmMotorID, false), kArmLimits, kArmFeedForwardGains,
                        new MotionConstraints(kZeroingSpeed, kArmRampRate,
                                new TrapezoidProfile.Constraints(kMaxVelocity,
                                        kMaxAcceleration),
                                kArmLowerLimit, kArmUpperLimit),
                        kArmGains,
                        new SimulationDetails(kArmGearboxRatio, kArmExternalRatio, kArmMOI,
                                kStandardDevs)),
                new SensorConfig(kArmUpperLimit, kArmLowerLimit, kArmPositionTolerance), kArmLength);
        public static final FlywheelConfig kWheelConfig = new FlywheelConfig(
                new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX,
                        Map.of(kWheelMotorID, false), kWheelLimits, null, null, null,
                        new SimulationDetails(kWheelGearboxRatio, kWheelExternalRatio,
                                kWheelMOI, kStandardDevs)),
                new SensorConfig(kLaserCANID, kCoralDetectDistance, kCoralDetectTolerance, kROI));
    }

    // TODO: #16 Find the actual values for Coral
    public static final class AlgaeConstants {
        private static final int kArmMotorID = 0;
        private static final int kWheelMotorID = 0;
        private static final int kLaserCANID = 0;
        private static final double kArmGearboxRatio = 0.0;
        private static final double kArmMOI = 0.0;
        private static final double kArmExternalRatio = 1.0;
        private static final double kWheelGearboxRatio = 0.0;
        private static final double kWheelMOI = 0.0;
        private static final double kWheelExternalRatio = 1.0;
        private static final double[] kStandardDevs = { 0.0, 0.1 };
        private static final double kArmLength = 0.0;
        private static final FeedForwardGains kArmFeedForwardGains = new FeedForwardGains(0, 0, 0, 0);
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
        private static final double kZeroingSpeed = 0;

        public static final AngularConfig kArmConfig = new AngularConfig(
                new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX,
                        Map.of(kArmMotorID, false), kArmLimits, kArmFeedForwardGains,
                        new MotionConstraints(kZeroingSpeed, kArmRampRate,
                                new TrapezoidProfile.Constraints(kMaxVelocity,
                                        kMaxAcceleration),
                                kArmLowerLimit, kArmUpperLimit),
                        kArmGains,
                        new SimulationDetails(kArmGearboxRatio, kArmExternalRatio, kArmMOI,
                                kStandardDevs)),
                new SensorConfig(kArmUpperLimit, kArmLowerLimit, kArmPositionTolerance), kArmLength);
        public static final FlywheelConfig kWheelConfig = new FlywheelConfig(
                new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX,
                        Map.of(kWheelMotorID, false), kWheelLimits, null, null, null,
                        new SimulationDetails(kWheelGearboxRatio, kWheelExternalRatio,
                                kWheelMOI, kStandardDevs)),
                new SensorConfig(kLaserCANID, kAlgaeDetectDistance, kAlgaeDetectTolerance, kROI));
    }
}
