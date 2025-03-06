package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.simulation.MockLaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.motor.MotorConfig;
import frc.lib.constants.motor.SensorConfig;
import frc.lib.constants.motor.SimulationDetails;
import frc.lib.constants.subsystem.SubsystemConstants;

public class EndEffectorSubsystem extends SubsystemBase {
    public enum FieldPosition {
        STARTING,
        DRIVING,
        HPSTATION,
        PROCESSOR,
        NET,
        L1,
        L2,
        L3,
        L4
    }

    protected final Map<FieldPosition, Double> POSITIONS;
    private final SensorConfig sensorConfig;
    private final MotorConfig armConfig;
    private final MechanismLigament2d mechanism;

    private final SparkFlex m_wheelMotor, m_armMotor;
    private final AbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_controller;

    private final ArmFeedforward m_feedforward;
    private final TrapezoidProfile m_profile;

    private final DCMotor m_wheelGearbox, m_armGearbox;
    private final SparkFlexSim m_wheelMotorSim, m_armMotorSim;
    private final FlywheelSim m_WheelSim;
    private final SingleJointedArmSim m_ArmSim;

    private final LaserCanInterface m_laser;

    private final EventLoop m_loop = new EventLoop();
    private final BooleanEvent hasGamePiece;

    public EndEffectorSubsystem(SubsystemConstants constants, Map<FieldPosition, Double> positions, MechanismLigament2d mechanism) {
        POSITIONS = new EnumMap<>(positions);
        this.mechanism = mechanism;

        // Configure Wheel Motor

        MotorConfig wheelConfig = constants.kWheelConfig.kMotorConfig;
        m_wheelMotor = new SparkFlex(wheelConfig.motorID, SparkFlex.MotorType.kBrushless);
        m_wheelMotor.configure(new SparkFlexConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(
                wheelConfig.kCurrentLimits.peakCurrentLimit,
                wheelConfig.kCurrentLimits.continuousCurrentLimit),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Arm Motor

        armConfig = constants.kWheelConfig.kMotorConfig;
        m_armMotor = new SparkFlex(armConfig.motorID, SparkFlex.MotorType.kBrushless);
        m_armMotor.configure(new SparkFlexConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(
                armConfig.kCurrentLimits.peakCurrentLimit, armConfig.kCurrentLimits.continuousCurrentLimit)
                .closedLoopRampRate(armConfig.kConstraints.kRampRate)
                .apply(armConfig.kGains.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_armMotor.getAbsoluteEncoder();
        m_controller = m_armMotor.getClosedLoopController();
        m_feedforward = new ArmFeedforward(
                armConfig.kFeedForwardGains.kS,
                armConfig.kFeedForwardGains.kG,
                armConfig.kFeedForwardGains.kV,
                armConfig.kFeedForwardGains.kA);
        m_profile = new TrapezoidProfile(armConfig.kConstraints.constraints);

        // Configure Simulation

        sensorConfig = constants.kWheelConfig.kSensorConfig;
        if (RobotBase.isSimulation()) {
            SimulationDetails wheelDetails = armConfig.kSimulation;
            m_wheelGearbox = DCMotor.getNeoVortex(1).withReduction(wheelDetails.kGearboxRatio);
            m_wheelMotorSim = new SparkFlexSim(m_wheelMotor, m_wheelGearbox);
            m_WheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(m_wheelGearbox, wheelDetails.kMOI,
                    wheelDetails.kExternalRatio), m_wheelGearbox, wheelDetails.kStandardDevs);

            SimulationDetails armDetails = armConfig.kSimulation;
            m_armGearbox = DCMotor.getNeoVortex(1).withReduction(armDetails.kGearboxRatio);
            m_armMotorSim = new SparkFlexSim(m_armMotor, m_armGearbox);

            m_ArmSim = new SingleJointedArmSim(m_armGearbox, armDetails.kExternalRatio, armDetails.kMOI,
                    constants.kArmConfig.kArmLength, armConfig.kConstraints.kLowerLimit,
                    armConfig.kConstraints.kUpperLimit, true, positions.get(FieldPosition.STARTING),
                    armDetails.kStandardDevs);

            m_laser = new MockLaserCan();
        } else {
            m_wheelGearbox = null;
            m_armGearbox = null;
            m_armMotorSim = null;
            m_wheelMotorSim = null;
            m_ArmSim = null;
            m_WheelSim = null;

            m_laser = new LaserCan(sensorConfig.laserCANID);
        }

        // Configure LaserCAN
        // Optionally initialise the settings of the LaserCAN, if you haven't already
        // done so in GrappleHook
        try {
            m_laser.setRangingMode(LaserCan.RangingMode.SHORT);
            m_laser.setRegionOfInterest(sensorConfig.kROI);
            m_laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
        hasGamePiece = new BooleanEvent(m_loop, () -> MathUtil.isNear(sensorConfig.kDetectDistance,
                m_laser.getMeasurement().distance_mm,
                sensorConfig.kDetectTolerance));
    }

    // Commands

    /**
     * Set the goal of the arm
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command aim(FieldPosition goal) {
        return runEnd(() -> {
            reachGoal(POSITIONS.get(goal));
            stopWheel();
        }, () -> {
            stop();
            stopWheel();
        }).until(atPosition(goal));
    }

    /**
     * Release the with arm at set position
     *
     * @param goal Goal in meters
     * @param fire Fire event
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command score(FieldPosition goal, BooleanEvent fire) {
        return runEnd(() -> {
            reachGoal(POSITIONS.get(goal));
            fire.ifHigh(() -> {
                setWheel(-0.5);
                if (m_laser instanceof MockLaserCan) {
                    ((MockLaserCan) m_laser).setMeasurementPartialSim(
                            LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT,
                            (int) (sensorConfig.kDetectDistance - (0.1 * sensorConfig.kDetectTolerance)), 0);
                }
            });
        }, () -> {
            stop();
            stopWheel();
        }).until(hasGamePiece.negate());
    }

    /**
     * Intake from human player station
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command intake(FieldPosition position) {
        return runEnd(() -> {
            reachGoal(POSITIONS.get(position));
            atPosition(position).ifHigh(() -> {
                setWheel(0.5);
                if (m_laser instanceof MockLaserCan) {
                    ((MockLaserCan) m_laser).setMeasurementPartialSim(
                            LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT,
                            (int) (sensorConfig.kDetectDistance - (0.1 * sensorConfig.kDetectTolerance)), 0);
                }
            });
        }, () -> {
            stop();
            stopWheel();
        }).until(hasGamePiece);
    }

    // Periodic

    /**
     * Update the subsystem.
     */
    @Override
    public void periodic() {
        m_loop.poll();
    }

    /**
     * Advance the simulation.
     */
    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our mechanism is doing
        // First, we set our "inputs" (voltages)
        m_ArmSim.setInput(m_armMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        m_WheelSim.setInput(m_wheelMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_ArmSim.update(0.020);
        m_WheelSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_armMotorSim.iterate(m_ArmSim.getVelocityRadPerSec(),
                RoboRioSim.getVInVoltage(),
                0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_wheelMotorSim.iterate(m_WheelSim.getAngularVelocityRadPerSec(),
                RoboRioSim.getVInVoltage(),
                0.020);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_ArmSim.getCurrentDrawAmps(),
                        m_WheelSim.getCurrentDrawAmps()));
    }

    // Controls

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public void reachGoal(double goal) {
        State setpoint = m_profile.calculate(TimedRobot.kDefaultPeriod,
                new State(m_encoder.getPosition(), m_encoder.getVelocity()),
                new State(goal, 0.0));
        double feedforward = m_feedforward.calculate(m_encoder.getPosition(), setpoint.velocity);
        m_controller.setReference(setpoint.velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    /**
     * Set the wheel speed
     *
     * @param speed Speed in meters per second
     */
    public void setWheel(double speed) {
        m_wheelMotor.set(speed);
    }

    /**
     * Stop the control loop and motor output.
     */
    public void stop() {
        double feedforward = m_feedforward.calculateWithVelocities(m_encoder.getPosition(), m_encoder.getVelocity(),
                0.0);
        m_controller.setReference(0.0, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    /**
     * Stop the wheel
     */
    public void stopWheel() {
        m_wheelMotor.set(0.0);
    }

    // Telemetry

    /**
     * Update telemetry, including the mechanism visualization.
     */
    public void updateTelemetry() {
        // Update elevator visualization with position
        mechanism.setAngle(m_encoder.getPosition());
    }

    // Triggers

    /**
     * Trigger when the arm is at a position
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.button.Trigger}
     */
    public BooleanEvent atPosition(FieldPosition goal) {
        return new BooleanEvent(m_loop, () -> MathUtil.isNear(POSITIONS.get(goal),
                m_encoder.getPosition(),
                armConfig.kConstraints.kPositionTolerance));
    }
}
