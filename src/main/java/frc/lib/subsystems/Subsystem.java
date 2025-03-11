package frc.lib.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.configurations.motor.MotionConstraints;
import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SimulationDetails;
import frc.lib.configurations.sensors.SensorConfig;
import frc.lib.configurations.subsystem.SubsystemConfig;

public abstract class Subsystem extends SubsystemBase implements ISubsystem {

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

    public static class FieldPositionValue implements Consumer<Double> {
        public Optional<Double> m_value = Optional.empty();

        @Override
        public void accept(Double value) {
            m_value = Optional.of(value);
        }
    }

    protected boolean kZeroed = false;

    protected final Map<FieldPosition, FieldPositionValue> POSITIONS;

    protected final SubsystemConfig constants;
    protected final MechanismLigament2d mechanism;

    private List<SparkBase> m_motors = new ArrayList<>();
    protected SparkBase m_motor;
    protected SparkBaseConfig m_sparkConfig;
    protected RelativeEncoder m_encoder;
    protected SparkClosedLoopController m_controller;

    protected TrapezoidProfile m_profile;

    private LaserCanInterface m_laser;
    private AbsoluteEncoder m_absoluteEncoder;
    private CANcoder m_canCoder;
    private CANcoderSimState m_canCoderSim;

    protected DCMotor m_gearbox;
    protected SparkSim m_motorSim;

    //private SimpleMotorFeedforward m_simpleFeedForward;
    //private DCMotorSim m_sim;

    protected final EventLoop m_loop = new EventLoop();
    protected Trigger m_forwardTrigger, m_reverseTrigger;

    // SysID Routine
    protected SysIdRoutine m_sysIdRoutine;

    public Subsystem(SubsystemConfig constants, Map<FieldPosition, FieldPositionValue> positions,
            MechanismLigament2d mechanism) {
        POSITIONS = positions;
        this.mechanism = mechanism;
        this.constants = constants;

        configureMotors();
        configureSensor();

        if (DriverStation.isTest()) {
            configureSysId();
        }
    }

    // Configuration
    /**
     * Configure the motors and controllers.
     */
    private void configureMotors() {
        m_sparkConfig = getSparkConfig();
        // Configure Motor
        MotorConfig config = constants.kMotorConfig;
        for (int id : config.motorIDs.keySet()) {
            SparkBase motor = getSpark(id);

            if (m_motors.isEmpty()) {
                motor.configure(
                        m_sparkConfig.apply(config.kGains.feedbackSensor(FeedbackSensor.kPrimaryEncoder))
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(config.kCurrentLimits.peakCurrentLimit,
                                        config.kCurrentLimits.continuousCurrentLimit)
                                .closedLoopRampRate(config.kConstraints.kRampRate),
                        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                m_motor = motor;
            } else {
                motor.configure(getSparkConfig().follow(m_motors.get(0)), ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
            }

            m_motors.add(motor);
        }

        SparkBase m_motor = m_motors.get(0);
        m_encoder = m_motor.getEncoder();
        m_controller = m_motor.getClosedLoopController();

        m_profile = new TrapezoidProfile(config.kConstraints.constraints);

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

        if (RobotBase.isSimulation()) {
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
                    m_loop.bind(() -> {
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
                    });
                } else {
                    m_laser = new LaserCan(config.canID);
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
                if (config.canID != 0) {
                    m_canCoder = new CANcoder(config.canID);
                    if (RobotBase.isSimulation()) {
                        m_canCoderSim = new CANcoderSimState(m_canCoder, ChassisReference.CounterClockwise_Positive);
                        m_canCoderSim.setMagnetHealth(MagnetHealthValue.Magnet_Green);
                        m_loop.bind(() -> {
                            m_canCoderSim.addPosition(Units.radiansToRotations(m_motorSim.getPosition()));
                            m_canCoderSim.setVelocity(Units.radiansToRotations(m_motorSim.getVelocity()));
                            m_canCoderSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
                        });
                    }
                    m_loop.bind(() -> {
                        if (m_canCoder != null && m_canCoder.isConnected()) {
                            StatusSignal<Angle> position = m_canCoder.getAbsolutePosition();
                            if (position.hasUpdated() && position.getStatus() == StatusCode.OK) {
                                m_encoder.setPosition(position.getValue().baseUnitMagnitude());
                            }
                        }
                    });
                } else {
                    m_absoluteEncoder = m_motor.getAbsoluteEncoder();
                    m_encoder.setPosition(m_absoluteEncoder.getPosition());

                    if (RobotBase.isSimulation()) {
                        m_motorSim.getAbsoluteEncoderSim()
                                .setPosition(POSITIONS.get(FieldPosition.STARTING).m_value.get());
                        m_loop.bind(() -> {
                            m_motorSim.getAbsoluteEncoderSim()
                                    .setPosition(m_motorSim.getPosition());
                        });
                    }
                }

                MotionConstraints constraints = constants.kMotorConfig.kConstraints;
                m_motor.configure(
                        new SparkFlexConfig().apply(new ClosedLoopConfig().positionWrappingEnabled(true)
                                .positionWrappingInputRange(constraints.kLowerLimit, constraints.kUpperLimit)),
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
                break;
            case LIMIT_SWITCH:
                if (RobotBase.isSimulation()) {
                    m_loop.bind(() -> {
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
                    });
                }
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
     * Configure the SysId routine.
     */
    protected abstract void configureSysId();

    // Getters

    private SparkBaseConfig getSparkConfig() {
        MotorConfig config = constants.kMotorConfig;
        switch (config.kControllerType) {
            case SPARK_MAX:
                return new SparkMaxConfig();
            case SPARK_FLEX:
                return new SparkMaxConfig();
            default:
                throw new IllegalArgumentException("Unknown controller type: " + config.kControllerType);
        }
    }

    private SparkBase getSpark(int id) {
        MotorConfig config = constants.kMotorConfig;
        switch (config.kControllerType) {
            case SPARK_MAX:
                return new SparkMax(id, MotorType.kBrushless);
            case SPARK_FLEX:
                return new SparkFlex(id, MotorType.kBrushless);
            default:
                throw new IllegalArgumentException("Unknown controller type: " + config.kControllerType);
        }
    }

    // Commands

    /**
     * Runs the SysId routine to tune the Arm
     *
     * @return SysId Routine command
     */
    public final Command runSysIdRoutine() {
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
            runMotor(POSITIONS.get(goal).m_value.get());
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
        return aim(goal).finallyDo(this::stop).until(sensorEvent(goal).negate());
    }

    /**
     * Intake from human player station
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command intake(FieldPosition position) {
        return aim(position).finallyDo(this::stop).until(sensorEvent(position));
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
    }

    // Controls

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position (or speed for wheels) to maintain
     */
    public abstract void runMotor(double goal);

    /**
     * Stop the control loop and motor output.
     */
    public abstract void stop();

    // Telemetry

    /**
     * Update telemetry, including the mechanism visualization.
     */
    protected abstract void updateTelemetry();

    /**
     * Get the mechanism position.
     *
     * @return Mechanism position
     */
    protected abstract double getPosition();

    /**
     * Get the mechanism velocity.
     *
     * @return Mechanism velocity
     */
    protected abstract double getVelocity();

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
            case LIMIT_SWITCH:
                return new BooleanEvent(m_loop, () -> MathUtil.isNear(POSITIONS.get(goal).m_value.get(),
                        m_encoder.getPosition(),
                        sensorConfig.kTolerance));
            default:
                throw new IllegalArgumentException("Unknown sensor type: " + sensorConfig.kType);
        }
    }
}
