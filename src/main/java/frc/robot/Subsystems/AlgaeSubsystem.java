package frc.robot.Subsystems;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
    // THESE ARE DUMMY VALUES!!!!! TODO: #8 Update algae pivot position values once
    // determined.
    public enum Position {
        STARTING(0.0),
        PROCESSOR(0.0),
        L2(40.0),
        L3(60.0),
        BARGE(100.0);

        private static final Map<Double, Position> lookup = new HashMap<Double, Position>();

        static {
            for (Position p : EnumSet.allOf(Position.class))
                lookup.put(p.getPosition(), p);
        }
        private double position;

        private Position(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        public static Position get(double position) {
            return lookup.get(position);
        }
    }

    private final DCMotor m_wheelGearbox = DCMotor.getNeoVortex(1);
    private final DCMotor m_armGearbox = DCMotor.getNeoVortex(1);

    private final SparkFlex m_wheel = new SparkFlex(14, SparkFlex.MotorType.kBrushless);
    private final SparkFlexSim m_wheelSim = new SparkFlexSim(m_wheel, m_wheelGearbox);
    private final SparkFlex m_arm = new SparkFlex(15, SparkFlex.MotorType.kBrushless);
    private final SparkFlexSim m_armSim = new SparkFlexSim(m_arm, m_armGearbox);
    private final AbsoluteEncoder m_encoder = m_arm.getAbsoluteEncoder();
    private final SparkClosedLoopController m_controller = m_arm.getClosedLoopController();

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            AlgaeConstants.kS,
            AlgaeConstants.kG,
            AlgaeConstants.kV,
            AlgaeConstants.kA);

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(AlgaeConstants.kMaxVelocity, AlgaeConstants.kMaxAcceleration));

    private final SingleJointedArmSim m_algaeArmSim = new SingleJointedArmSim(m_armGearbox, 9, 0, 0, 0, 0, true, 0, 0.0,
            0.1);
    private final FlywheelSim m_algaeWheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(m_wheelGearbox, 1, 1), m_wheelGearbox, 1, 0.1);

    private final LaserCanInterface m_laser;

    private final BooleanEvent hasAlgae;
    private final EventLoop m_loop = new EventLoop();

    public AlgaeSubsystem() {
        m_wheel.configure(new SparkFlexConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(
                AlgaeConstants.wheelLimits.peakCurrentLimit, AlgaeConstants.wheelLimits.continuousCurrentLimit),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_arm.configure(new SparkFlexConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(
                AlgaeConstants.angleLimits.peakCurrentLimit, AlgaeConstants.angleLimits.continuousCurrentLimit)
                .closedLoopRampRate(AlgaeConstants.kRampRate)
                .apply(AlgaeConstants.gains.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).outputRange(-1, 1)),
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            m_laser = new MockLaserCan();
        } else {
            m_laser = new LaserCan(0);
        }

        // Optionally initialise the settings of the LaserCAN, if you haven't already
        // done so in GrappleHook
        try {
            m_laser.setRangingMode(LaserCan.RangingMode.SHORT);
            m_laser.setRegionOfInterest(AlgaeConstants.roi);
            m_laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }

        hasAlgae = new BooleanEvent(m_loop, () -> MathUtil.isNear(AlgaeConstants.kAlgaeDetectDistance,
            m_laser.getMeasurement().distance_mm,
            AlgaeConstants.kAlgaeDetectTolerance));
    }
    
    // Commands

    /**
     * Set the goal of the pivot
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command aimAlgae(Position goal) {
        return runEnd(() -> {
            reachGoal(goal.getPosition());
            stopWheel();
        }, () -> {
            stop();
            stopWheel();
        }).until(atPosition(goal));
    }

    /**
     * Score algae in barge or processor
     *
     * @param bargeOrProcessor Goal in meters
     * @param fire             Fire event
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command scoreAlgae(Position bargeOrProcessor, BooleanEvent fire) {
        return runEnd(() -> {
            reachGoal(bargeOrProcessor.getPosition());
            fire.ifHigh(() -> setWheel(-0.5));
        }, () -> {
            stop();
            stopWheel();
        }).until(hasAlgae.negate());
    }

    /**
     * Intake algae from reef
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command intakeAlgae(Position level) {
        return runEnd(() -> {
            reachGoal(level.getPosition());
            atPosition(level).ifHigh(() -> setWheel(0.5));
        }, () -> {
            stop();
            stopWheel();
        }).until(hasAlgae);
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
        m_algaeArmSim.setInput(m_armSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        m_algaeWheelSim.setInput(m_wheelSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_algaeArmSim.update(0.020);
        m_algaeWheelSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_armSim.iterate(m_algaeArmSim.getVelocityRadPerSec(),
                RoboRioSim.getVInVoltage(),
                0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_wheelSim.iterate(m_algaeWheelSim.getAngularVelocityRadPerSec(),
                RoboRioSim.getVInVoltage(),
                0.020);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_algaeArmSim.getCurrentDrawAmps(),
                        m_algaeWheelSim.getCurrentDrawAmps()));
        
        // TODO: simulate laserCAN
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
        m_wheel.set(speed);
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
        m_wheel.set(0.0);
    }

    // Triggers

    /**
     * Trigger when the arm is at a position
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.button.Trigger}
     */
    public BooleanEvent atPosition(Position goal) {
        return new BooleanEvent(m_loop, () -> MathUtil.isNear(goal.getPosition(),
                m_encoder.getPosition(),
                AlgaeConstants.kPositionTolerance));
    }
}