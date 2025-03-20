package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    // THESE ARE DUMMY VALUES!!!!! TODO: #11 Update elevator position values once
    // determined.
    public enum Position {
        ZERO(0.0),
        PROCESSOR(0.0),
        HPSTATION(0.0),
        L1(20.0),
        L2(30.0),
        L3(40.0),
        L4(50.0),
        BARGE(100.0);

        private static final Map<Double, Position> lookup = new HashMap<>();

        static {
            for (Position p : EnumSet.allOf(Position.class)) {
                lookup.put(p.getPosition(), p);
            }
        }
        private final double position;

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

    // Zeroed state
    private boolean kZeroed = false;

    // This gearbox represents a gearbox containing 2 Neos
    // Either we have 2 gear boxes(1 for each Neo) or 1 gear box with 2 Neos//
    // private final DCMotor m_elevatorGearbox =
    // DCMotor.getNEO(2).withReduction(15.0);
    /*
     * // Standard classes for controlling our elevator
     * private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
     * ElevatorConstants.kElevatorkS,
     * ElevatorConstants.kElevatorkG,
     * ElevatorConstants.kElevatorkV,
     * ElevatorConstants.kElevatorkA);
     */
    private final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity,
                    ElevatorConstants.kMaxAcceleration));

    // SparkMax and Encoder
    private final SparkMax m_motor = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(10, MotorType.kBrushless);
    // private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor,
    // m_elevatorGearbox);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();

    // Limit Switches
    private final DigitalInput m_limitSwitchLow = new DigitalInput(0);
    // private final DigitalInput m_limitSwitchHigh = new DigitalInput(2);
    // private DIOSim m_limitSwitchLowSim = null;
    // private DIOSim m_limitSwitchHighSim = null;
    // public Trigger atMax = new
    // Trigger(m_limitSwitchLow::get).onTrue(this.run(this::stop));

    /*
     * public Trigger atMin = new
     * Trigger(m_limitSwitchLow::get).onFalse(this.runOnce(() -> {
     * stop();
     * m_encoder.setPosition(0.0);
     * kZeroed = true;
     * }));
     */

    private static final double kConversionFactor = 1;// 6.73;

    private double target = 0.0;
    /*
     * // Simulation classes help us simulate what's going on, including gravity.
     * private final ElevatorSim m_elevatorSim = new ElevatorSim(
     * m_elevatorGearbox,
     * ElevatorConstants.kElevatorGearing,
     * ElevatorConstants.kCarriageMass,
     * ElevatorConstants.kElevatorDrumRadius,
     * ElevatorConstants.kMinElevatorHeight.in(Meters),
     * ElevatorConstants.kMaxElevatorHeight.in(Meters),
     * true,
     * ElevatorConstants.kStartingHeightSim.in(Meters),
     * 0.01,
     * 0.0);
     */

    /*
     * // SysId Routine and seutp
     * // Mutable holder for unit-safe voltage values, persisted to avoid
     * reallocation.
     * private final MutVoltage m_appliedVoltage = Volts.mutable(0);
     * // Mutable holder for unit-safe linear distance values, persisted to avoid
     * // reallocation.
     * private final MutDistance m_distance = Meters.mutable(0);
     * // Mutable holder for unit-safe linear velocity values, persisted to avoid
     * // reallocation.
     * private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
     * // SysID Routine
     * private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
     * // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
     * new SysIdRoutine.Config(Volts.per(Second).of(1),
     * Volts.of(7),
     * Seconds.of(10)),
     * new SysIdRoutine.Mechanism(
     * // Tell SysId how to plumb the driving voltage to the motor(s).
     * m_motor::setVoltage,
     * // Tell SysId how to record a frame of data for each motor on the mechanism
     * // being
     * // characterized.
     * log -> {
     * // Record a frame for the shooter motor.
     * log.motor("elevator")
     * .voltage(
     * m_appliedVoltage.mut_replace(
     * m_motor.getAppliedOutput()
     * RobotController
     * .getBatteryVoltage(),
     * Volts))
     * .linearPosition(m_distance.mut_replace(
     * m_encoder.getPosition(),
     * Meters)) // Records Height in Meters via
     * // SysIdRoutineLog.linearPosition
     * .linearVelocity(m_velocity.mut_replace(
     * m_encoder.getVelocity(),
     * MetersPerSecond)); // Records velocity
     * // in MetersPerSecond
     * // via
     * // SysIdRoutineLog.linearVelocity
     * },
     * this));
     */

    /**
     * Subsystem constructor.
     */
    public ElevatorSubsystem() {
        // kZeroed = false;
        m_motor.configure(
                new SparkMaxConfig().inverted(true).smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
                        .apply(new EncoderConfig()
                                .positionConversionFactor(kConversionFactor))
                        // .velocityConversionFactor())
                        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
                        .apply(new ClosedLoopConfig()
                                .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi,
                                        ElevatorConstants.kElevatorKd)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(-1, 1)),
                ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_motor2.configure(new SparkMaxConfig().follow(m_motor, true), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator visualization, select Network Tables -> SmartDashboard
        // -> Elevator Sim

        /*
         * if (RobotBase.isSimulation()) {
         * m_limitSwitchLowSim = new DIOSim(m_limitSwitchLow);
         * SmartDashboard.putData("Elevator Low Limit Switch", m_limitSwitchLow);
         * 
         * m_limitSwitchHighSim = new DIOSim(m_limitSwitchHigh);
         * SmartDashboard.putData("Elevator Low Limit Switch", m_limitSwitchHigh);
         * }
         */

        // this.setDefaultCommand(zeroElevator());
    }

    // Commands

    /**
     * Runs the SysId routine to tune the Arm
     *
     * @return SysId Routine command
     */
    /*
     * public Command runSysIdRoutine() {
     * return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
     * .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
     * .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
     * .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
     * .andThen(Commands.print("DONE"));
     * }
     */
    /*
     * /**
     * Set the goal of the elevator
     *
     * @param goal Goal in meters
     * 
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    /*
     * public Command setGoal(Position goal) {
     * return runEnd(() -> reachGoal(goal.getPosition()), this::stop)
     * .until(new Trigger(() -> MathUtil.isNear(goal.getPosition(),
     * getCarriageHeight(),
     * ElevatorConstants.kPositionTolerance)));
     * }
     */

    /**
     * Zero the elevator
     *
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public void zeroElevator() {
        // return runEnd(() -> {
        if (kZeroed) {
            reachGoal(0.0);
        } else {
            // double feedforward =
            // m_feedforward.calculateWithVelocities(m_encoder.getVelocity(),
            // ElevatorConstants.kZeroingSpeed);
            // m_motor.stopMotor();
            m_controller.setReference(ElevatorConstants.kZeroingSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            // feedforward);
        }
    }// , this::stop).until(atMin);

    /**
     * Advance the simulation.
     */
    // @Override
    /*
     * public void simulationPeriodic() {
     * if (m_motorSim.getRelativeEncoderSim().getPosition() <= 0.02) {
     * m_limitSwitchLowSim.setValue(true);
     * } else {
     * m_limitSwitchLowSim.setValue(false);
     * }
     * 
     * if (m_motorSim.getRelativeEncoderSim().getPosition() >= 200.0) {
     * m_limitSwitchHighSim.setValue(true);
     * } else {
     * m_limitSwitchHighSim.setValue(false);
     * }
     * // In this method, we update our simulation of what our elevator is doing
     * // First, we set our "inputs" (voltages)
     * m_elevatorSim.setInput(m_motorSim.getAppliedOutput() *
     * RoboRioSim.getVInVoltage());
     * 
     * // Next, we update it. The standard loop time is 20ms.
     * m_elevatorSim.update(0.020);
     * 
     * // Finally, we set our simulated encoder's readings and simulated battery
     * // voltage
     * m_motorSim.iterate(
     * Elevator
     * .convertDistanceToRotations(
     * Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
     * .per(Second).in(RPM),
     * RoboRioSim.getVInVoltage(),
     * 0.020);
     * 
     * // SimBattery estimates loaded battery voltages
     * RoboRioSim.setVInVoltage(
     * BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.
     * getCurrentDrawAmps()));
     * }
     */

    // Control

    public void set(Position goal) {
        target = goal.position;
    }

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public void reachGoal(double goal) {
        /*
         * State setpoint = m_profile.calculate(TimedRobot.kDefaultPeriod,
         * new State(m_encoder.getPosition(), m_encoder.getVelocity()),
         * new State(goal, 0.0));
         */

        // double feedforward =
        // m_feedforward.calculateWithVelocities(m_encoder.getVelocity(),
        // setpoint.velocity);
        // m_motor.stopMotor();
        m_controller.setReference(goal, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }

    /**
     * Stop the control loop and motor output.
     */
    public void stop() {
        // double feedforward =
        // m_feedforward.calculateWithVelocities(m_encoder.getVelocity(), 0.0);
        // m_controller.setReference(0.0, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
        // feedforward);
        m_motor.stopMotor();
    }

    /**
     * Get the height in meters.
     *
     * @return Height in meters
     */
    public double getCarriageHeight() {
        return (m_encoder.getPosition());
    }

    /**
     * The velocity of the elevator in meters per second.
     *
     * @return velocity in meters per second
     */
    public double getCarriageVelocity() {
        return m_encoder.getVelocity();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Elevator Zeroed", !m_limitSwitchLow.get());
        SmartDashboard.putNumber("Elevator Height", getCarriageHeight());
        SmartDashboard.putNumber("Elevator Velocity", getCarriageVelocity());
        SmartDashboard.putNumber("Elevator Output Voltage", m_motor.getAppliedOutput() * m_motor.getBusVoltage());

        if (getCarriageHeight() < 0.0 || !m_limitSwitchLow.get()) {
            m_encoder.setPosition(0.0);
        }

        if (MathUtil.isNear(target, getCarriageHeight(), 1)) {
            if (target != 0.0) {
                reachGoal(0.26);
            } else {
                stop();
            }
        } else if (getCarriageHeight() < target) {
            reachGoal(3.5);
        } else if (getCarriageHeight() > target) {
            reachGoal(-2.5);
        } else {
            stop();
        }
    }
}