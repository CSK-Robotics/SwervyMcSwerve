package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.controls.CurrentLimits;
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
}
