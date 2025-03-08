package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.simulation.MockLaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.configurations.motor.MotionConstraints;
import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SensorConfig;
import frc.lib.configurations.motor.SimulationDetails;
import frc.lib.configurations.subsystem.AngularConfig;
import frc.lib.configurations.subsystem.FlywheelConfig;
import frc.lib.configurations.subsystem.LinearConfig;
import frc.lib.configurations.subsystem.SubsystemConfig;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;

public class Subsystem extends SubsystemBase implements ISubsystem {

    public enum FieldPosition {
        STARTING,
        DRIVING,
        HPSTATION,
        PROCESSOR,
        NET,
        CAGE,
        L1,
        L2,
        L3,
        L4
    }

    private boolean kZeroed = false;

    protected final Map<FieldPosition, Double> POSITIONS;
    private final SubsystemConfig constants;
    private final MechanismLigament2d mechanism;

    private List<SparkBase> m_motors = new ArrayList<>();
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_controller;

    private TrapezoidProfile m_profile;

    private AbsoluteEncoder m_absoluteEncoder;
    private LaserCanInterface m_laser;

    private DCMotor m_gearbox;
    private SparkSim m_motorSim;

    private ArmFeedforward m_armFeedForward;
    private SingleJointedArmSim m_armSim;

    private SimpleMotorFeedforward m_flywheelFeedForward;
    private FlywheelSim m_flywheelSim;

    private ElevatorFeedforward m_elevatorFeedForward;
    private ElevatorSim m_elevatorSim;

    private final EventLoop m_loop = new EventLoop();
    protected Trigger m_forwardTrigger, m_reverseTrigger;

    // SysId Routine and seutp
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
    // SysID Routine
    private SysIdRoutine m_sysIdRoutine;

    public Subsystem(SubsystemConfig constants, Map<FieldPosition, Double> positions,
            MechanismLigament2d mechanism) {
        POSITIONS = positions;
        this.mechanism = mechanism;
        this.constants = constants;

        configureMotors();
        configureSensor();

        switch (constants.kType) {
            case FLYWHEEL:
                configureFlywheel((FlywheelConfig) constants);
                break;
            case ARM:
                configureArm((AngularConfig) constants);
                break;
            case ELEVETOR:
                configureElevator((LinearConfig) constants);
                break;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }

        if (DriverStation.isTest()) {
            configureSysId();
        }
    }

    // Configuration

    /**
     * Configure the motors and controllers.
     */
    private void configureMotors() {
        // Configure Motor
        MotorConfig config = constants.kMotorConfig;
        for (int id : config.motorIDs.keySet()) {
            SparkBase motor;
            SparkBaseConfig sparkConfig;
            switch (config.kControllerType) {
                case SPARK_MAX:
                    motor = new SparkMax(id, MotorType.kBrushless);
                    sparkConfig = new SparkMaxConfig();
                    break;
                case SPARK_FLEX:
                    motor = new SparkFlex(id, MotorType.kBrushless);
                    sparkConfig = new SparkMaxConfig();
                    break;
                default:
                    throw new IllegalArgumentException("Unknown controller type: " + config.kControllerType);
            }

            if (m_motors.isEmpty()) {
                sparkConfig.apply(config.kGains).idleMode(IdleMode.kBrake).smartCurrentLimit(
                        config.kCurrentLimits.peakCurrentLimit, config.kCurrentLimits.continuousCurrentLimit)
                        .closedLoopRampRate(config.kConstraints.kRampRate);
            } else {
                sparkConfig.follow(m_motors.get(0));
            }

            motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_motors.add(motor);
        }

        SparkBase m_motor = m_motors.get(0);
        m_encoder = m_motor.getEncoder();
        m_controller = m_motor.getClosedLoopController();

        m_profile = new TrapezoidProfile(config.kConstraints.constraints);

        if (RobotBase.isSimulation()) {
            int motorCount = m_motors.size();
            SimulationDetails sim = constants.kMotorConfig.kSimulation;
            switch (config.kMotorType) {
                case NEO:
                    m_gearbox = DCMotor.getNEO(motorCount);
                    break;
                case NEO550:
                    m_gearbox = DCMotor.getNeo550(motorCount);
                    break;
                case NEOVORTEX:
                    m_gearbox = DCMotor.getNeoVortex(motorCount);
                    break;
                case BAG:
                    m_gearbox = DCMotor.getBag(motorCount);
                    break;
                case MINICIM:
                    m_gearbox = DCMotor.getMiniCIM(motorCount);
                    break;
                case CIM:
                    m_gearbox = DCMotor.getCIM(motorCount);
                    break;
                case PRO775:
                    m_gearbox = DCMotor.getVex775Pro(motorCount);
                    break;
                default:
                    throw new IllegalArgumentException("Unknown motor type: " + config.kMotorType);
            }
            m_gearbox = m_gearbox.withReduction(sim.kGearboxRatio);
            switch (config.kControllerType) {
                case SPARK_MAX:
                    m_motorSim = new SparkMaxSim((SparkMax) m_motor, m_gearbox);
                    break;
                case SPARK_FLEX:
                    m_motorSim = new SparkFlexSim((SparkFlex) m_motor, m_gearbox);
                    break;
                default:
                    throw new IllegalArgumentException("Unknown controller type: " + config.kControllerType);
            }
            m_motorSim = new SparkSim(m_motor, m_gearbox);
        } else {
            m_gearbox = null;
            m_motorSim = null;
        }
    }

    /**
     * Configure the sensor.
     */
    private void configureSensor() {
        SparkBase m_motor = m_motors.get(0);
        SensorConfig config = constants.kSensorConfig;
        switch (config.kType) {
            case LASER:
                if (RobotBase.isSimulation()) {
                    m_laser = new MockLaserCan();
                } else {
                    m_laser = new LaserCan(config.laserCANID);
                }

                // Configure LaserCAN
                // Optionally initialise the settings of the LaserCAN, if you haven't already
                // done so in GrappleHook
                try {
                    m_laser.setRangingMode(LaserCan.RangingMode.SHORT);
                    m_laser.setRegionOfInterest(config.kROI);
                    m_laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
                } catch (ConfigurationFailedException e) {
                    System.out.println("Configuration failed! " + e);
                }
                break;
            case ABSOLUTE:
                m_absoluteEncoder = m_motor.getAbsoluteEncoder();
                m_motor.configure(
                        new SparkFlexConfig()
                                .apply(new ClosedLoopConfig().feedbackSensor(FeedbackSensor.kAbsoluteEncoder))
                                .apply(new SoftLimitConfig().forwardSoftLimitEnabled(config.kHasForwardLimit)
                                        .forwardSoftLimit(config.kForwardLimit)
                                        .reverseSoftLimitEnabled(config.kHasReverseLimit)
                                        .reverseSoftLimit(config.kReverseLimit)),
                        ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

                m_forwardTrigger = new Trigger(m_loop,
                        () -> MathUtil.isNear(config.kForwardLimit, m_absoluteEncoder.getPosition(), config.kTolerance))
                        .debounce(0.2).onTrue(this.run(this::stop));
                m_reverseTrigger = new Trigger(m_loop,
                        () -> MathUtil.isNear(config.kReverseLimit, m_absoluteEncoder.getPosition(), config.kTolerance))
                        .debounce(0.2).onTrue(this.run(() -> {
                            stop();
                            m_encoder.setPosition(0.0);
                            kZeroed = true;
                        }));
                if (RobotBase.isSimulation()) {
                    m_motorSim.getAbsoluteEncoderSim().setPosition(POSITIONS.get(FieldPosition.STARTING));
                }
                break;
            case LIMIT_SWITCH:
                m_motor.configure(
                        new SparkFlexConfig()
                                .apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(config.kHasForwardLimit)
                                        .forwardLimitSwitchType(Type.kNormallyOpen)
                                        .reverseLimitSwitchEnabled(config.kHasReverseLimit)
                                        .reverseLimitSwitchType(Type.kNormallyOpen).setSparkMaxDataPortConfig()),
                        ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

                m_reverseTrigger = new Trigger(m_loop, m_motor.getReverseLimitSwitch()::isPressed).debounce(0.2)
                        .onTrue(this.run(this::stop));
                m_forwardTrigger = new Trigger(m_loop, m_motor.getForwardLimitSwitch()::isPressed).debounce(0.2)
                        .onTrue(this.run(() -> {
                            stop();
                            m_encoder.setPosition(0.0);
                            kZeroed = true;
                        }));
                break;
            default:
                throw new IllegalArgumentException("Unknown sensor type: " + config.kType);
        }
    }

    /**
     * Configure the flywheel.
     *
     * @param config Flywheel configuration
     */
    private void configureFlywheel(FlywheelConfig config) {
        // Configure Flywheel Specifics
        MotorConfig motorConfig = config.kMotorConfig;

        m_flywheelFeedForward = new SimpleMotorFeedforward(motorConfig.kFeedForwardGains.kS,
                motorConfig.kFeedForwardGains.kV, motorConfig.kFeedForwardGains.kA, Robot.kDefaultPeriod);

        if (RobotBase.isSimulation()) {
            SimulationDetails sim = config.kMotorConfig.kSimulation;
            m_flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(m_gearbox, sim.kMOI,
                    sim.kExternalRatio), m_gearbox, sim.kStandardDevs);
        } else {
            m_flywheelSim = null;
        }
    }

    /**
     * Configure the arm.
     *
     * @param constants Arm configuration
     */
    private void configureArm(AngularConfig constants) {
        // Configure Arm Specifics
        MotorConfig config = constants.kMotorConfig;

        m_armFeedForward = new ArmFeedforward(
                config.kFeedForwardGains.kS,
                config.kFeedForwardGains.kG,
                config.kFeedForwardGains.kV,
                config.kFeedForwardGains.kA);

        if (RobotBase.isSimulation()) {
            SimulationDetails sim = config.kSimulation;
            m_armSim = new SingleJointedArmSim(m_gearbox, sim.kExternalRatio, sim.kMOI,
                    constants.kArmLength, config.kConstraints.kLowerLimit,
                    config.kConstraints.kUpperLimit, true, POSITIONS.get(FieldPosition.STARTING),
                    sim.kStandardDevs);
        } else {
            m_armSim = null;
        }
    }

    /**
     * Configure the elevator.
     *
     * @param constants Elevator configuration
     */
    private void configureElevator(LinearConfig constants) {
        // Configure Elevator Specifics
        MotorConfig config = constants.kMotorConfig;

        m_elevatorFeedForward = new ElevatorFeedforward(
                config.kFeedForwardGains.kS,
                config.kFeedForwardGains.kG,
                config.kFeedForwardGains.kV,
                config.kFeedForwardGains.kA);

        if (RobotBase.isSimulation()) {
            SimulationDetails sim = config.kSimulation;
            m_elevatorSim = new ElevatorSim(m_gearbox, sim.kExternalRatio, sim.kMOI,
                    constants.kDrumRadius, config.kConstraints.kLowerLimit,
                    config.kConstraints.kUpperLimit, true, POSITIONS.get(FieldPosition.STARTING),
                    sim.kStandardDevs);
        } else {
            m_elevatorSim = null;
        }
    }

    /**
     * Configure the SysId routine.
     */
    private void configureSysId() {
        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(Volts.per(Second).of(1),
                        Volts.of(7),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motor(s).
                        m_motors.get(0)::setVoltage,
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor(mechanism.getName())
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_motors.get(0).getAppliedOutput()
                                                            * RobotController
                                                                    .getBatteryVoltage(),
                                                    Volts))
                                    .linearPosition(m_distance.mut_replace(
                                            m_encoder.getPosition(),
                                            Meters)) // Records Height in Meters via
                                                     // SysIdRoutineLog.linearPosition
                                    .linearVelocity(m_velocity.mut_replace(
                                            m_encoder.getVelocity(),
                                            MetersPerSecond)); // Records velocity
                                                               // in MetersPerSecond
                                                               // via
                                                               // SysIdRoutineLog.linearVelocity
                        },
                        this));
    }

    // Commands

    /**
     * Runs the SysId routine to tune the Arm
     *
     * @return SysId Routine command
     */
    public Command runSysIdRoutine() {
        return (m_sysIdRoutine.dynamic(Direction.kForward).until(m_forwardTrigger))
                .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(m_reverseTrigger))
                .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(m_forwardTrigger))
                .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(m_reverseTrigger))
                .andThen(Commands.print("DONE"));
    }

    /**
     * Set the goal of the arm
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command aim(FieldPosition goal) {
        return run(() -> {
            runMotor(POSITIONS.get(goal));
        });
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
            fire.ifHigh(() -> {
                if (constants.kType == SubsystemConfig.Type.FLYWHEEL) {
                    runMotor(-POSITIONS.get(goal));
                } else {
                    runMotor(POSITIONS.get(goal));
                }
            });
        }, () -> {
            stop();
        }).until(sensorEvent(goal));
    }

    /**
     * Intake from human player station
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command intake(FieldPosition position) {
        return runEnd(() -> {
                runMotor(POSITIONS.get(position));
            }, this::stop).until(sensorEvent(position));
    }

    /**
     * Zero the subsystem
     *
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command zero() {
        return run(() -> {
            if (kZeroed) {
                runMotor(0.0);
            } else {
                MotionConstraints constraints = constants.kMotorConfig.kConstraints;
                double feedforward;
                switch (constants.kType) {
                    case FLYWHEEL:
                        feedforward = m_flywheelFeedForward.calculateWithVelocities(m_encoder.getVelocity(),
                                constraints.kZeroingSpeed);
                        break;
                    case ARM:
                        feedforward = m_armFeedForward.calculateWithVelocities(getPosition(), m_encoder.getVelocity(),
                                constraints.kZeroingSpeed);
                        break;
                    case ELEVETOR:
                        feedforward = m_elevatorFeedForward.calculateWithVelocities(m_encoder.getVelocity(),
                                constraints.kZeroingSpeed);
                        break;
                    default:
                        throw new IllegalArgumentException(
                                "Unknown configuration type: " + constants.getClass().getName());
                }
                m_controller.setReference(constraints.kZeroingSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                        feedforward);
            }
        });
    }

    // Periodic

    /**
     * Update the subsystem.
     */
    @Override
    public void periodic() {
        m_loop.poll();
        updateTelemetry();
    }

    /**
     * Advance the simulation.
     */
    @Override
    public void simulationPeriodic() {
        switch (constants.kType) {
            case FLYWHEEL:
                m_flywheelSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
                m_flywheelSim.update(Robot.kDefaultPeriod);
                m_motorSim.iterate(m_flywheelSim.getAngularVelocityRadPerSec(),
                        RoboRioSim.getVInVoltage(),
                        0.020);
                RoboRioSim.setVInVoltage(
                        BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps()));
                break;
            case ARM:
                m_armSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
                m_armSim.update(Robot.kDefaultPeriod);
                m_motorSim.iterate(m_armSim.getVelocityRadPerSec(),
                        RoboRioSim.getVInVoltage(),
                        0.020);
                RoboRioSim.setVInVoltage(
                        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
                break;
            case ELEVETOR:
                m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
                m_elevatorSim.update(Robot.kDefaultPeriod);
                m_motorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(),
                        RoboRioSim.getVInVoltage(),
                        0.020);
                RoboRioSim.setVInVoltage(
                        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
                break;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }

        SimulationDetails sim = constants.kMotorConfig.kSimulation;
        switch (constants.kSensorConfig.kType) {
            case LASER:
                MockLaserCan laser = (MockLaserCan) m_laser;
                if (m_motorSim.getPosition() > 16.0) {
                    laser.setMeasurementPartialSim(
                            LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT,
                            (int) (constants.kSensorConfig.kDetectDistance
                            - (0.1 * constants.kSensorConfig.kTolerance)),
                            0);
                } else {
                    laser.setMeasurementFullSim(null);
                }
                break;
            case ABSOLUTE:
                m_motorSim.getAbsoluteEncoderSim().setPosition(m_motorSim.getPosition() / (sim.kGearboxRatio * sim.kExternalRatio));
                break;
            case LIMIT_SWITCH:
                if (m_motorSim.getPosition() > constants.kMotorConfig.kConstraints.kLowerLimit) {
                    m_motorSim.getReverseLimitSwitchSim().setPressed(false);
                } else {
                    m_motorSim.getReverseLimitSwitchSim().setPressed(true);
                }

                if (m_motorSim.getPosition() < constants.kMotorConfig.kConstraints.kUpperLimit) {
                    m_motorSim.getForwardLimitSwitchSim().setPressed(false);
                } else {
                    m_motorSim.getForwardLimitSwitchSim().setPressed(true);
                }
                break;
            default:
                throw new IllegalArgumentException("Unknown sensor type: " + constants.kSensorConfig.kType);
        }
    }

    // Controls

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position (or speed for flywheels) to maintain
     */
    public void runMotor(double goal) {
        State setpoint = m_profile.calculate(Robot.kDefaultPeriod,
                new State(m_encoder.getPosition(), m_encoder.getVelocity()),
                new State(goal, 0.0));

        double feedforward;
        switch (constants.kType) {
            case FLYWHEEL:
                feedforward = m_flywheelFeedForward.calculateWithVelocities(m_encoder.getVelocity(), goal);
                break;
            case ARM:
                feedforward = m_armFeedForward.calculateWithVelocities(getPosition(), m_encoder.getVelocity(),
                        setpoint.velocity);
                break;
            case ELEVETOR:
                feedforward = m_elevatorFeedForward.calculateWithVelocities(m_encoder.getVelocity(), setpoint.velocity);
                break;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }
        m_controller.setReference(setpoint.velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    /**
     * Stop the control loop and motor output.
     */
    public void stop() {
        double feedforward;
        switch (constants.kType) {
            case FLYWHEEL:
                feedforward = m_flywheelFeedForward.calculateWithVelocities(m_encoder.getVelocity(), 0.0);
                break;
            case ARM:
                feedforward = m_armFeedForward.calculateWithVelocities(getPosition(), m_encoder.getVelocity(), 0.0);
                break;
            case ELEVETOR:
                feedforward = m_elevatorFeedForward.calculateWithVelocities(m_encoder.getVelocity(), 0.0);
                break;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }
        m_controller.setReference(0.0, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    // Telemetry

    /**
     * Update telemetry, including the mechanism visualization.
     */
    private void updateTelemetry() {
        // Update mechanism visualization with position
        switch (constants.kType) {
            case FLYWHEEL:
                break;
            case ARM:
                mechanism.setAngle(getPosition());
                break;
            case ELEVETOR:
                LinearConfig config = (LinearConfig) constants;
                mechanism.setLength((config.kBaseHeight - config.kCarriageGroundOffset) - getPosition());
                break;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }
    }

    /**
     * Get the mechanism position.
     *
     * @return Mechanism position
     */
    public double getPosition() {
        switch (constants.kType) {
            case ARM:
                return m_encoder.getPosition();
            case ELEVETOR:
                return (m_encoder.getPosition() * 2) + ElevatorConstants.kElevatorConfig.kCarriageGroundOffset;
            case FLYWHEEL:
            default:
                return 0.0;
        }
    }

    /**
     * Get the mechanism velocity.
     *
     * @return Mechanism velocity
     */
    public double getVelocity() {
        switch (constants.kType) {
            case ARM:
                return m_encoder.getVelocity();
            case ELEVETOR:
                return m_encoder.getVelocity() * 2;
            case FLYWHEEL:
            default:
                return 0.0;
        }
    }

    // Triggers

    /**
     * Trigger when the sensor detects the goal.
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.button.Trigger}
     */
    public BooleanEvent sensorEvent(FieldPosition goal) {
        SensorConfig sensorConfig = constants.kSensorConfig;
        switch (sensorConfig.kType) {
            case LASER:
                return new BooleanEvent(m_loop, () -> MathUtil.isNear(sensorConfig.kDetectDistance,
                        m_laser.getMeasurement().distance_mm,
                        sensorConfig.kTolerance));
            case ABSOLUTE:
                return new BooleanEvent(m_loop, () -> MathUtil.isNear(POSITIONS.get(goal),
                        m_absoluteEncoder.getPosition(),
                        sensorConfig.kTolerance));
            case LIMIT_SWITCH:
                return new BooleanEvent(m_loop, () -> MathUtil.isNear(POSITIONS.get(goal),
                        m_encoder.getPosition(),
                        sensorConfig.kTolerance));
            default:
                throw new IllegalArgumentException("Unknown sensor type: " + sensorConfig.kType);
        }
    }
}
