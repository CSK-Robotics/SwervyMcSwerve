package frc.lib.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/* Contains values and required settings for common COTS swerve modules. */
public class SwerveInstances {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKF;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    public final boolean canCoderInvert;

    private final DCMotor driveMotorConstants;
    private final DCMotor angleMotorConstants;

    public SwerveInstances(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP,
            double angleKI, double angleKD, double angleKF, boolean driveMotorInvert, boolean angleMotorInvert,
            boolean canCoderInvert) {
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKF = angleKF;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.canCoderInvert = canCoderInvert;

        this.driveMotorConstants = DCMotor.getNeoVortex(1)
            .withReduction(this.driveGearRatio);
        this.angleMotorConstants = DCMotor.getNeoVortex(1)
            .withReduction(this.angleGearRatio);
    }

    /** Swerve Drive Specialties - MK3 Module */
    public static SwerveInstances SDSMK3(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = false;
        boolean canCoderInvert = false;
        return new SwerveInstances(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF,
                driveMotorInvert, angleMotorInvert, canCoderInvert);
    }

    /** Swerve Drive Specialties - MK4 Module */
    public static SwerveInstances SDSMK4(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = false;
        boolean canCoderInvert = false;
        return new SwerveInstances(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF,
                driveMotorInvert, angleMotorInvert, canCoderInvert);
    }

    /** Swerve Drive Specialties - MK4i Module */
    public static SwerveInstances SDSMK4i(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 0.3;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        boolean canCoderInvert = false;
        return new SwerveInstances(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF,
                driveMotorInvert, angleMotorInvert, canCoderInvert);
    }

    /* Drive Gear Ratios for all supported modules */
    public class driveGearRatios {
        /* SDS MK3 */
        /** SDS MK3 - 8.16 : 1 */
        public static final double SDSMK3_Standard = (8.16 / 1.0);
        /** SDS MK3 - 6.86 : 1 */
        public static final double SDSMK3_Fast = (6.86 / 1.0);

        /* SDS MK4 */
        /** SDS MK4 - 8.14 : 1 */
        public static final double SDSMK4_L1 = (8.14 / 1.0);
        /** SDS MK4 - 6.75 : 1 */
        public static final double SDSMK4_L2 = (6.75 / 1.0);
        /** SDS MK4 - 6.12 : 1 */
        public static final double SDSMK4_L3 = (6.12 / 1.0);
        /** SDS MK4 - 5.14 : 1 */
        public static final double SDSMK4_L4 = (5.14 / 1.0);

        /* SDS MK4i */
        /** SDS MK4i - 8.14 : 1 */
        public static final double SDSMK4i_L1 = (8.14 / 1.0);
        /** SDS MK4i - 6.75 : 1 */
        public static final double SDSMK4i_L2 = (6.75 / 1.0);
        /** SDS MK4i - 6.12 : 1 */
        public static final double SDSMK4i_L3 = (6.12 / 1.0);
    }

    public double drivePositionConversionFactor() {
        return wheelDiameter * Math.PI / driveGearRatio;
    }

    public double driveVelocityConversionFactor() {
        return drivePositionConversionFactor() * 60;
    }

    public double anglePositionConversionFactor() {
        return 360 / angleGearRatio;
    }

    public double angleVelocityConversionFactor() {
        return anglePositionConversionFactor() * 60;
    }

    public double driveFreeSpeed() {
        return (driveMotorConstants.freeSpeedRadPerSec / 2 * Math.PI) * drivePositionConversionFactor();
    }

    public double angleFreeSpeed() {
        return (angleMotorConstants.freeSpeedRadPerSec / 2 * Math.PI) * anglePositionConversionFactor();
    }
}