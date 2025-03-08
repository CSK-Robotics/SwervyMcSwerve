package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Millimeters;

import java.util.Map;

import frc.lib.Subsystem.FieldPosition;
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
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;

public final class Constants {
	public static final double stickDeadband = 0.05;

	public static final class Swerve {
		public static final Pose2d startingPose = new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
				new Rotation2d(0));
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

	// TODO: #12 Find the actual values for the camera
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

	public class AutonomousData {
		public int desiredAprilTag;
		public int pipelineID;
		public String starterPathPlanner;
		public double areaAprilTag;
		public double direction;

		public AutonomousData(int desiredAprilTag, int pipelineID, String starterPathPlanner,
				double areaAprilTag, double direction) {
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

	// TODO: #14 Find the actual values for the path planner
	public static final class AutoConstants {
		// Load the RobotConfig from the GUI settings. You should probably
		// store this in your Constants file
		public static RobotConfig pathplannerConfig;

		public static final double kMaxSpeedMetersPerSecond = 2;
		public static final double kMaxAccelerationMetersPerSecondSquared = 1;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

		public static final double XY_kP = 5;
		public static final double XY_kI = 0;
		public static final double XY_kD = 0;

		public static final double THETA_kP = 6.2;
		public static final double THETA_kI = 0;
		public static final double THETA_kD = 0;

		// Motion profilied robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond,
				kMaxAngularSpeedRadiansPerSecondSquared);
	}

	// TODO: #13 Find the actual values for elevator
	public static class ElevatorConstants extends LinearConfig {
		// IDs
		private static final int kElevatorMotorID = 14;
		private static final int kSecondaryElevatorMotorID = 15;

		// Physical Constants
		private static final double kElevatorCarriageGroundOffset = Units.inchesToMeters(20.0);
		private static final double kElevatorHeight = Units.inchesToMeters(34.0);
		private static final double kElevatorDrumRadius = Units.inchesToMeters(1.757);

		// Gains
		private static final ClosedLoopConfig kElevatorGains = new ClosedLoopConfig()
				.pidf(26.722, 0, 1.6047, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
		private static final FeedForwardGains kElevatorFeedForwardGains = new FeedForwardGains(0.01964, 0.91274, 3.894,
				0.173);

		// Simulation Details
		private static final double[] kStandardDevs = { 0.0, 0.1 };
		private static final double kElevatorWeight = Units.lbsToKilograms(12.5);
		private static final SimulationDetails kElevatorSimulationDetails = new SimulationDetails(15.0, 1.0,
				kElevatorWeight,
				kStandardDevs);

		// Constraints
		private static final double kZeroingSpeed = 0.25;
		private static final double kTravelDistance = Units.inchesToMeters(26);
		private static final MotionConstraints kElevatorConstraints = new MotionConstraints(kZeroingSpeed,
				new TrapezoidProfile.Constraints(Units.inchesToMeters(20.0), Units.inchesToMeters(40.0)),
				ElevatorSubsystem.POSITIONS.get(FieldPosition.STARTING), kTravelDistance);
		private static final CurrentLimits kElevatorCurrentLimits = new CurrentLimits(25, 40, 0.1, true);

		// Sensor Constants
		private static final double kPositionTolerance = Units.inchesToMeters(0.5);
		private static final SensorConfig kElevatorSensorConfig = new SensorConfig(false, true, kPositionTolerance);

		private ElevatorConstants() {
			super(new MotorConfig(MotorType.NEO, ControllerType.SPARK_MAX,
					Map.of(kElevatorMotorID, false, kSecondaryElevatorMotorID, true), kElevatorCurrentLimits,
					kElevatorFeedForwardGains, kElevatorConstraints, kElevatorGains, kElevatorSimulationDetails),
					kElevatorSensorConfig, kElevatorDrumRadius, kElevatorCarriageGroundOffset, kElevatorHeight);
		}

		public static final ElevatorConstants kElevatorConfig = new ElevatorConstants();
	}

	// TODO: #15 Find the actual values for Coral
	public static final class CoralConstants {
		private static final double[] kStandardDevs = { 0.0, 0.1 };

		public static final class ArmConfig extends AngularConfig {
			// Arm IDs
			private static final int kArmMotorID = 16;

			// Physical Constants
			private static final double kCoralArmLength = Units.inchesToMeters(2.0);
			private static final double kCoralArmWeight = Units.lbsToKilograms(3.8);

			// Gains
			private static final ClosedLoopConfig kArmGains = new ClosedLoopConfig()
					.pidf(26.722, 0, 1.6047, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
			private static final FeedForwardGains kArmFeedForwardGains = new FeedForwardGains(0.01964, 0.91274, 3.894,
					0.173);

			// Simulation Details
			private static final SimulationDetails kArmSimulationDetails = new SimulationDetails(9.0, 1.0,
					kCoralArmWeight, kStandardDevs);

			// Constraints
			private static final double kZeroingSpeed = 0.25;
			private static final double kMaxAngle = Units.degreesToRadians(180.0);
			private static final MotionConstraints kArmConstraints = new MotionConstraints(kZeroingSpeed,
					new TrapezoidProfile.Constraints(Units.degreesToRadians(90.0), Units.degreesToRadians(180.0)),
					ClimberSubsystem.POSITIONS.get(FieldPosition.STARTING), kMaxAngle);
			private static final CurrentLimits kArmCurrentLimits = new CurrentLimits(0, 40, 0.1, true);

			// Sensor Constants
			private static final double kPositionTolerance = Units.degreesToRadians(0.5);
			private static final SensorConfig kArmSensorConfig = new SensorConfig(kMaxAngle, 0, kPositionTolerance);

			// ArmConfig
			private ArmConfig() {
				super(new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX, Map.of(kArmMotorID, false),
						kArmCurrentLimits, kArmFeedForwardGains, kArmConstraints, kArmGains, kArmSimulationDetails),
						kArmSensorConfig, kCoralArmLength);
			}
		}

		public static final class WheelConfig extends FlywheelConfig {
			// Wheel IDs
			private static final int kWheelMotorID = 17;
			private static final int kLaserCANID = 18;

			// Physical Constants
			private static final double kCoralWheelDiameter = Units.inchesToMeters(4.0);
			private static final double kCoralWheelWeight = Units.lbsToKilograms(1.2);

			// Constraints
			private static final CurrentLimits kWheelLimits = new CurrentLimits(25, 40, 0.0, true);

			// Sensor Constants
			private static final RegionOfInterest kROI = new RegionOfInterest(8, 8, 16, 16);
			private static final double kCoralDetectTolerance = Millimeters.fromBaseUnits(Units.inchesToMeters(0.25));
			private static final double kCoralDetectDistance = Millimeters.fromBaseUnits(Units.inchesToMeters(0.5));
			private static final SensorConfig kWheelSensorConfig = new SensorConfig(kLaserCANID, kCoralDetectDistance,
					kCoralDetectTolerance, kROI);

			// Simulation Details
			private static final SimulationDetails kWheelSimulationDetails = new SimulationDetails(1.0, 1.0,
					kCoralWheelWeight, kStandardDevs);

			// WheelConfig
			public WheelConfig() {
				super(new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX, Map.of(kWheelMotorID, false),
						kWheelLimits, null, null, null, kWheelSimulationDetails), kWheelSensorConfig,
						kCoralWheelDiameter);
			}
		}

		public static final ArmConfig kArmConfig = new ArmConfig();
		public static final WheelConfig kWheelConfig = new WheelConfig();
	}

	// TODO: #16 Find the actual values for Coral
	public static final class AlgaeConstants {
		private static final double[] kStandardDevs = { 0.0, 0.1 };

		public static final class ArmConfig extends AngularConfig {
			// Arm IDs
			private static final int kArmMotorID = 19;

			// Physical Constants
			private static final double kAlgaeArmLength = Units.inchesToMeters(14.0);
			private static final double kAlgaeArmWeight = Units.lbsToKilograms(2.5);

			// Gains
			private static final ClosedLoopConfig kArmGains = new ClosedLoopConfig()
					.pidf(26.722, 0, 1.6047, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
			private static final FeedForwardGains kArmFeedForwardGains = new FeedForwardGains(0.01964, 0.91274, 3.894,
					0.173);

			// Simulation Details
			private static final SimulationDetails kArmSimulationDetails = new SimulationDetails(25.0, 1.0,
					kAlgaeArmWeight,
					kStandardDevs);

			// Constraints
			private static final double kZeroingSpeed = 0.25;
			private static final double kMaxAngle = Units.degreesToRadians(180.0);
			private static final MotionConstraints kArmConstraints = new MotionConstraints(kZeroingSpeed,
					new TrapezoidProfile.Constraints(Units.degreesToRadians(90.0), Units.degreesToRadians(180.0)),
					ClimberSubsystem.POSITIONS.get(FieldPosition.STARTING), kMaxAngle);
			private static final CurrentLimits kArmCurrentLimits = new CurrentLimits(35, 40, 0.1, true);

			// Sensor Constants
			private static final double kPositionTolerance = Units.degreesToRadians(0.5);
			private static final SensorConfig kArmSensorConfig = new SensorConfig(kMaxAngle, 0, kPositionTolerance);

			// ArmConfig
			private ArmConfig() {
				super(new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX, Map.of(kArmMotorID, false),
						kArmCurrentLimits, kArmFeedForwardGains, kArmConstraints, kArmGains, kArmSimulationDetails),
						kArmSensorConfig, kAlgaeArmLength);
			}
		}

		public static final class WheelConfig extends FlywheelConfig {
			// Wheel IDs
			private static final int kWheelMotorID = 20;
			private static final int kLaserCANID = 21;

			// Physical Constants
			private static final double kAlgaeWheelDiameter = Units.inchesToMeters(4.0);
			private static final double kAlgaeWheelWeight = Units.lbsToKilograms(1.2);

			// Constraints
			private static final CurrentLimits kWheelLimits = new CurrentLimits(20, 35, 0.0, true);

			// Sensor Constants
			private static final RegionOfInterest kROI = new RegionOfInterest(8, 8, 16, 16);
			private static final double kAlgaeDetectTolerance = Millimeters.fromBaseUnits(Units.inchesToMeters(0.25));
			private static final double kAlgaeDetectDistance = Millimeters.fromBaseUnits(Units.inchesToMeters(0.5));
			private static final SensorConfig kWheelSensorConfig = new SensorConfig(kLaserCANID, kAlgaeDetectDistance,
					kAlgaeDetectTolerance, kROI);

			// Simulation Details
			private static final SimulationDetails kWheelSimulationDetails = new SimulationDetails(1.0, 1.0,
					kAlgaeWheelWeight, kStandardDevs);

			// WheelConfig
			public WheelConfig() {
				super(new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX, Map.of(kWheelMotorID, false),
						kWheelLimits, null, null, null, kWheelSimulationDetails), kWheelSensorConfig,
						kAlgaeWheelDiameter);
			}
		}

		public static final ArmConfig kArmConfig = new ArmConfig();
		public static final WheelConfig kWheelConfig = new WheelConfig();
	}

	// TODO: #17 Find the actual values for Climber
	public static final class ClimberConstants extends AngularConfig {
		// IDs
		private static final int kArmMotorID = 22;
		private static final int kSecondaryArmMotorID = 23;

		// Physical Constants
		private static final double kClimberArmLength = Units.inchesToMeters(5.0);
		private static final double kClimberArmWeight = Units.lbsToKilograms(0.8);

		// Gains
		private static final ClosedLoopConfig kArmGains = new ClosedLoopConfig()
				.pidf(26.722, 0, 1.6047, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
		private static final FeedForwardGains kArmFeedForwardGains = new FeedForwardGains(0.01964, 0.91274, 3.894,
				0.173);

		// Simulation Details
		private static final double[] kStandardDevs = { 0.0, 0.1 };
		private static final SimulationDetails kArmSimulationDetails = new SimulationDetails(225.0, 1.0,
				kClimberArmWeight, kStandardDevs);

		// Constraints
		private static final double kZeroingSpeed = 0.25;
		private static final double kMaxAngle = Units.degreesToRadians(270.0);
		private static final MotionConstraints kArmConstraints = new MotionConstraints(kZeroingSpeed,
				new TrapezoidProfile.Constraints(Units.degreesToRadians(90.0), Units.degreesToRadians(180.0)),
				ClimberSubsystem.POSITIONS.get(FieldPosition.STARTING), kMaxAngle);
		private static final CurrentLimits kArmCurrentLimits = new CurrentLimits(30, 40, 0.1, true);

		// Sensor Constants
		private static final double kPositionTolerance = Units.degreesToRadians(0.5);
		private static final SensorConfig kArmSensorConfig = new SensorConfig(kMaxAngle, 0, kPositionTolerance);

		// ClimberConfig
		private ClimberConstants() {
			super(new MotorConfig(MotorType.NEO, ControllerType.SPARK_MAX,
					Map.of(kArmMotorID, false, kSecondaryArmMotorID, true), kArmCurrentLimits,
					kArmFeedForwardGains, kArmConstraints, kArmGains, kArmSimulationDetails),
					kArmSensorConfig, kClimberArmLength);
		}
		public static final ClimberConstants kClimberConfig = new ClimberConstants();
	}
}
