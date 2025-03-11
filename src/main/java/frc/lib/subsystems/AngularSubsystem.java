package frc.lib.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.configurations.motor.MotionConstraints;
import frc.lib.configurations.motor.MotorConfig;
import frc.lib.configurations.motor.SimulationDetails;
import frc.lib.configurations.subsystem.AngularConfig;
import frc.robot.Robot;

public class AngularSubsystem extends Subsystem {
    private final AngularConfig constants;

    private ArmFeedforward m_armFeedForward;
    private SingleJointedArmSim m_armSim;
    private SimpleMotorFeedforward m_simpleFeedForward;
    private DCMotorSim m_sim;
    // SysId Routine and seutp
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    protected final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    protected final MutAngle m_angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    protected final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

    public AngularSubsystem(AngularConfig constants, Map<FieldPosition, FieldPositionValue> positions,
            MechanismLigament2d mech) {
        super(constants, positions, mech);
        this.constants = constants;

        // Configure Arm Specifics
        MotorConfig config = constants.kMotorConfig;

        double conversionFactor = (2 * Math.PI) / config.kSimulation.kGearboxRatio;
        m_sparkConfig.apply(new EncoderConfig().positionConversionFactor(conversionFactor)
                .velocityConversionFactor(conversionFactor * 60));
        m_motor.configure(m_sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_armFeedForward = new ArmFeedforward(
                config.kFeedForwardGains.kS,
                config.kFeedForwardGains.kG,
                config.kFeedForwardGains.kV,
                config.kFeedForwardGains.kA);

        m_simpleFeedForward = new SimpleMotorFeedforward(config.kFeedForwardGains.kS,
                config.kFeedForwardGains.kV, config.kFeedForwardGains.kA, Robot.kDefaultPeriod);

        if (RobotBase.isSimulation()) {
            SimulationDetails sim = config.kSimulation;
            m_armSim = new SingleJointedArmSim(m_gearbox, sim.kExternalRatio,
                    sim.kMass * Math.pow(constants.kRadius / 2, 2),
                    constants.kRadius, config.kConstraints.kLowerLimit,
                    config.kConstraints.kUpperLimit, true, POSITIONS.get(FieldPosition.STARTING).m_value.get(),
                    sim.kStandardDevs);
            m_loop.bind(() -> {
                m_armSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
                m_armSim.update(Robot.kDefaultPeriod);
                m_motorSim.iterate(m_armSim.getVelocityRadPerSec(),
                        RoboRioSim.getVInVoltage(),
                        0.020);
                RoboRioSim.setVInVoltage(
                        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
            });

            m_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(m_gearbox,
                    sim.kMass * Math.pow(constants.kRadius / 2, 2), sim.kExternalRatio), m_gearbox, sim.kStandardDevs);
            m_loop.bind(() -> {
                m_sim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
                m_sim.update(Robot.kDefaultPeriod);
                RoboRioSim.setVInVoltage(
                        BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps()));
            });
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
                                    .angularPosition(m_angle.mut_replace(
                                            m_encoder.getPosition(),
                                            Radians)) // Records Height in Meters via
                                                      // SysIdRoutineLog.linearPosition
                                    .angularVelocity(m_velocity.mut_replace(
                                            m_encoder.getVelocity(),
                                            RadiansPerSecond)); // Records velocity
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
        return aim(goal).finallyDo(() -> {
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
                switch (constants.kAngularType) {
                    case HORIZONTAL:
                        feedforward = m_simpleFeedForward.calculateWithVelocities(m_encoder.getVelocity(),
                                constraints.kZeroingSpeed);
                        break;
                    case VERTICAL:
                        feedforward = m_armFeedForward.calculateWithVelocities(getPosition(), m_encoder.getVelocity(),
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
        switch (constants.kAngularType) {
            case HORIZONTAL:
                feedforward = m_simpleFeedForward.calculateWithVelocities(m_encoder.getVelocity(), setpoint.velocity);
                output = goal;
                break;
            case VERTICAL:
                feedforward = m_armFeedForward.calculateWithVelocities(getPosition(), m_encoder.getVelocity(),
                        setpoint.velocity);
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
        switch (constants.kAngularType) {
            case HORIZONTAL:
                feedforward = m_simpleFeedForward.calculateWithVelocities(m_encoder.getVelocity(), 0.0);
                break;
            case VERTICAL:
                feedforward = m_armFeedForward.calculateWithVelocities(getPosition(), m_encoder.getVelocity(), 0.0);
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
        switch (constants.kAngularType) {
            case HORIZONTAL:
                break;
            case VERTICAL:
                mechanism.setAngle(Rotation2d.fromRadians(getPosition()));
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
        return m_encoder.getPosition();
    }

    /**
     * Get the mechanism velocity.
     *
     * @return Mechanism velocity
     */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }
}
