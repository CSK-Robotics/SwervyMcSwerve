package frc.lib.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.configurations.motor.MotionConstraints;
import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SimulationDetails;
import frc.lib.configurations.subsystem.LinearConfig;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class LinearSubsystem extends Subsystem {
    private LinearConfig constants;
    private ElevatorFeedforward m_elevatorFeedForward;
    private ElevatorSim m_elevatorSim;
    private SimpleMotorFeedforward m_simpleFeedForward;
    private DCMotorSim m_sim;
    private FlywheelSim m_wheelSim;

    // SysId Routine and seutp
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    protected final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    protected final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    protected final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    public LinearSubsystem(LinearConfig constants, Map<FieldPosition, FieldPositionValue> positions,
            MechanismLigament2d mech) {
        super(constants, positions, mech);
        this.constants = constants;

        // Configure Elevator Specifics
        MotorConfig config = constants.kMotorConfig;

        double conversionFactor = (constants.kRadius * 2 * Math.PI) / config.kSimulation.kGearboxRatio;
        m_sparkConfig.apply(new EncoderConfig().positionConversionFactor(conversionFactor)
                .velocityConversionFactor(conversionFactor * 60));
        m_motor.configure(m_sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        switch (constants.kLinearType) {
            case ELEVETOR:
                m_elevatorFeedForward = new ElevatorFeedforward(
                        config.kFeedForwardGains.kS,
                        config.kFeedForwardGains.kG,
                        config.kFeedForwardGains.kV,
                        config.kFeedForwardGains.kA);
                break;
            case FLYWHEEL:
            case DRIVE:
            case SIMPLE:
                m_simpleFeedForward = new SimpleMotorFeedforward(config.kFeedForwardGains.kS,
                        config.kFeedForwardGains.kV, config.kFeedForwardGains.kA, Robot.kDefaultPeriod);
                break;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }

        m_loop.bind(this::updateTelemetry);

        if (RobotBase.isSimulation()) {
            configureSimulation();
        }
    }

    private void configureSimulation() {
        // Obtains the default instance of the simulation world, which is a Crescendo
        // Arena.
        SimulatedArena.getInstance();

        MotorConfig config = constants.kMotorConfig;
        SimulationDetails sim = config.kSimulation;

        switch (constants.kLinearType) {
            case ELEVETOR:
                m_elevatorSim = new ElevatorSim(m_gearbox, sim.kExternalRatio, sim.kMass,
                        constants.kRadius, config.kConstraints.kLowerLimit,
                        config.kConstraints.kUpperLimit, true, POSITIONS.get(FieldPosition.STARTING).m_value.get(),
                        sim.kStandardDevs);
                m_loop.bind(() -> {
                    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
                    m_elevatorSim.update(Robot.kDefaultPeriod);
                    m_motorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(),
                            RoboRioSim.getVInVoltage(),
                            0.020);
                    RoboRioSim.setVInVoltage(
                            BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
                });
                break;
            case FLYWHEEL:
                m_wheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(m_gearbox,
                        sim.kMass * Math.pow(constants.kRadius / 2, 2), sim.kExternalRatio), m_gearbox,
                        sim.kStandardDevs);
                m_loop.bind(() -> {
                    m_wheelSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
                    m_wheelSim.update(Robot.kDefaultPeriod);
                    m_motorSim.iterate(m_wheelSim.getAngularVelocityRadPerSec(),
                            RoboRioSim.getVInVoltage(),
                            0.020);
                    RoboRioSim.setVInVoltage(
                            BatterySim.calculateDefaultBatteryLoadedVoltage(m_wheelSim.getCurrentDrawAmps()));
                });
                break;
            case DRIVE:
            case SIMPLE:
                m_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(m_gearbox,
                        sim.kMass * Math.pow(constants.kRadius / 2, 2), sim.kExternalRatio), m_gearbox,
                        sim.kStandardDevs);
                m_loop.bind(() -> {
                    m_sim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
                    m_sim.update(Robot.kDefaultPeriod);
                    RoboRioSim.setVInVoltage(
                            BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps()));
                });
                break;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }
    }

    /**
     * Configure the SysId routine.
     */
    @Override
    protected void configureSysId() {
        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(Volts.per(Second).of(1),
                        Volts.of(7),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motor(s).
                        m_motor::setVoltage,
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor(mechanism.getName())
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    m_motor.getAppliedOutput()
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

    /**
     * Release the with arm at set position
     *
     * @param goal Goal in meters
     * @param fire Fire event
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command score(FieldPosition goal, BooleanEvent fire) {
        return runEnd(() -> {
            switch (constants.kLinearType) {
                case ELEVETOR:
                case DRIVE:
                    runMotor(POSITIONS.get(goal).m_value.get());
                    break;
                case FLYWHEEL:
                    runMotor(-POSITIONS.get(goal).m_value.get());
                case SIMPLE:
                    fire.ifHigh(() -> {
                        runMotor(-POSITIONS.get(goal).m_value.get());
                    });
                    break;
                default:
                    throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
            }
        }, () -> {
            stop();
        }).until(sensorEvent(goal).negate());
    }

    /**
     * Zero the subsystem
     *
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    @Override
    public Command zero() {
        return run(() -> {
            super.zero();
            if (!kZeroed) {
                MotionConstraints constraints = constants.kMotorConfig.kConstraints;
                double feedforward;
                switch (constants.kLinearType) {
                    case DRIVE:
                    case SIMPLE:
                    case FLYWHEEL:
                        feedforward = m_simpleFeedForward.calculateWithVelocities(m_encoder.getVelocity(),
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

    // Controls

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position (or speed for wheels) to maintain
     */
    public void runMotor(double goal) {
        State setpoint = m_profile.calculate(Robot.kDefaultPeriod,
                new State(m_encoder.getPosition(), m_encoder.getVelocity()),
                new State(goal, 0.0));

        double feedforward, output;
        switch (constants.kLinearType) {
            case DRIVE:
            case SIMPLE:
            case FLYWHEEL:
                feedforward = m_simpleFeedForward.calculateWithVelocities(m_encoder.getVelocity(), goal);
                output = goal;
                break;
            case ELEVETOR:
                feedforward = m_elevatorFeedForward.calculateWithVelocities(m_encoder.getVelocity(), setpoint.velocity);
                output = setpoint.velocity;
                break;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }
        m_controller.setReference(output, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    /**
     * Stop the control loop and motor output.
     */
    public void stop() {
        double feedforward;
        switch (constants.kLinearType) {
            case DRIVE:
            case SIMPLE:
            case FLYWHEEL:
                feedforward = m_simpleFeedForward.calculateWithVelocities(m_encoder.getVelocity(), 0.0);
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
    @Override
    protected void updateTelemetry() {
        // Update mechanism visualization with position
        switch (constants.kLinearType) {
            case FLYWHEEL:
            case SIMPLE:
            case DRIVE:
                break;
            case ELEVETOR:
                mechanism.setLength((constants.kBaseHeight - constants.kPositionOffset) - getPosition());
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
        switch (constants.kLinearType) {
            case FLYWHEEL:
            case DRIVE:
            case SIMPLE:
                return m_encoder.getPosition();
            case ELEVETOR:
                return (m_encoder.getPosition() * 2) + ElevatorConstants.kElevatorConfig.kPositionOffset;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }
    }

    /**
     * Get the mechanism velocity.
     *
     * @return Mechanism velocity
     */
    public double getVelocity() {
        switch (constants.kLinearType) {
            case FLYWHEEL:
            case DRIVE:
            case SIMPLE:
                return m_encoder.getVelocity();
            case ELEVETOR:
                return m_encoder.getVelocity() * 2;
            default:
                throw new IllegalArgumentException("Unknown configuration type: " + constants.getClass().getName());
        }
    }
}
