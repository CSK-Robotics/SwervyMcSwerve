package frc.lib.configurations.subsystem;

import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.Map;

import frc.lib.configurations.motor.CurrentLimits;
import frc.lib.configurations.motor.MotionConstraints;
import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.MotorConfig.ControllerType;
import frc.lib.configurations.motor.MotorConfig.MotorType;
import frc.lib.configurations.sensors.SensorConfig;
import frc.lib.subsystems.Subsystem.FieldPosition;
import frc.lib.swerve.SwerveInstances;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;

public class ModuleConfig {
	// Swerve Module Type
	public static final SwerveInstances instanceConstants = SwerveInstances
			.SDSMK4i(SwerveInstances.driveGearRatios.SDSMK4i_L2);

	public final class SwerveAngularConfig extends AngularConfig {
		// Gains
		private static final ClosedLoopConfig kGains = new ClosedLoopConfig().pidf(0.05, 0.0, 0.0, 0.0)
				.outputRange(-1, 1);

		// Constraints
		public static final double timeToMaxAngularVelocity = 5.0; // seconds
		private static final double kMaxAngle = Units.degreesToRadians(180.0);
		private static final MotionConstraints kConstraints = new MotionConstraints(0.0,
				new TrapezoidProfile.Constraints(Units.degreesToRadians(90.0),
						Units.degreesToRadians(180.0)),
				SwerveModule.AngleSubsystem.POSITIONS.get(FieldPosition.STARTING).m_value.get(),
				kMaxAngle);
		private static final CurrentLimits kCurrentLimits = new CurrentLimits(20, 40, 0.1, true);

		// Sensor Constants
		private static final double kPositionTolerance = Units.degreesToRadians(0.01);

		// ArmConfig
		private SwerveAngularConfig(int motorID, boolean invert, int sensorID) {
			super(AngularType.HORIZONTAL, new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX,
					Map.of(motorID, invert),
					kCurrentLimits, null, kConstraints, kGains, null),
					new SensorConfig(sensorID, kMaxAngle, -kMaxAngle,
							kPositionTolerance));
		}
	}

	public final class SwerveWheelConfig extends LinearConfig {
		// Physical Constants
		private static final double kSwerveWheelDiameter = Units.inchesToMeters(4.0);

		// Gains
		private static final ClosedLoopConfig kGains = new ClosedLoopConfig()
				.pidf(0.04, 0.0, 0.0, 1 / 565 /* Kv=565 for NEO Vortex (look up on website) */);

		// Constraints
		public static final double kMaxSpeed = DCMotor.getNeoVortex(1).withReduction(instanceConstants.driveGearRatio)
				.getSpeed(Constants.ROBOT_WEIGHT * 9.8 * instanceConstants.frictionCoefficient,
						Constants.PDH.getVoltage()) * kSwerveWheelDiameter / 2; // meters per second
		public static final double timeToMaxVelocity = 3.6576; // seconds
		private static final CurrentLimits kCurrentLimits = new CurrentLimits(35, 60, 0.1, true);
		private static final MotionConstraints kConstraints = new MotionConstraints(0.0,
				new TrapezoidProfile.Constraints(kMaxSpeed, kMaxSpeed / timeToMaxVelocity),
				-Double.MAX_VALUE,
				Double.MAX_VALUE);

		// WheelConfig
		public SwerveWheelConfig(int motorID, boolean invert) {
			super(LinearType.DRIVE, new MotorConfig(MotorType.NEOVORTEX, ControllerType.SPARK_FLEX,
					Map.of(motorID, invert),
					kCurrentLimits, null, kConstraints, kGains, null),
					null, kSwerveWheelDiameter);
		}
	}

	public final String kName;

	public final AngularConfig kAngularConfig;
	public final LinearConfig kWheelConfig;

	/**
	 * Swerve Module Constants to be used when creating swerve modules.
	 * 
	 * @param driveMotorID
	 * @param angleMotorID
	 * @param canCoderID
	 * @param angleOffset
	 * @param canBus
	 */
	public ModuleConfig(String name, int driveMotorID, int angleMotorID, int canCoderID,
			boolean driveMotorInvert) {
		this.kName = name;
		kAngularConfig = new SwerveAngularConfig(angleMotorID, false, canCoderID);
		kWheelConfig = new SwerveWheelConfig(driveMotorID, driveMotorInvert);
	}
}
