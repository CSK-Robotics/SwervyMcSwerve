package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.math.RobotMath.Elevator;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    // private boolean kZeroed;

    // This gearbox represents a gearbox containing 1 Neo
    // Either we have 2 gear boxes(1 for each Neo) or 1 gear box with 2 Neos//
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

    // Standard classes for controlling our elevator
    ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            ElevatorConstants.kElevatorkS,
            ElevatorConstants.kElevatorkG,
            ElevatorConstants.kElevatorkV,
            ElevatorConstants.kElevatorkA);

    private final SparkMax m_motor = new SparkMax(12, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);

    private final DigitalInput m_limitSwitchLow = new DigitalInput(1);
    private final DigitalInput m_limitSwitchHigh = new DigitalInput(2);
    private DIOSim m_limitSwitchLowSim = null;
    private DIOSim m_limitSwitchHighSim = null;
    public Trigger atMax = new Trigger(m_limitSwitchLow::get).onTrue(this.run(this::stop));
    public Trigger atMin = new Trigger(m_limitSwitchHigh::get).onTrue(this.run(() -> {
        stop();
        zeroElevator();
    }));

    private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kElevatorKp,
            ElevatorConstants.kElevatorKi,
            ElevatorConstants.kElevatorKd,
            new Constraints(ElevatorConstants.kMaxVelocity,
                    ElevatorConstants.kMaxAcceleration));

    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            ElevatorConstants.kElevatorGearing,
            ElevatorConstants.kCarriageMass,
            ElevatorConstants.kElevatorDrumRadius,
            ElevatorConstants.kMinElevatorHeight.in(Meters),
            ElevatorConstants.kMaxElevatorHeight.in(Meters),
            true,
            ElevatorConstants.kStartingHeightSim.in(Meters),
            0.01,
            0.0);

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
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
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
                        log.motor("elevator")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                m_motor.getAppliedOutput()
                                                        * RobotController
                                                                .getBatteryVoltage(),
                                                Volts))
                                .linearPosition(m_distance.mut_replace(
                                        getHeightMeters(),
                                        Meters)) // Records Height in Meters via
                                                 // SysIdRoutineLog.linearPosition
                                .linearVelocity(m_velocity.mut_replace(
                                        getVelocityMetersPerSecond(),
                                        MetersPerSecond)); // Records velocity
                                                           // in MetersPerSecond
                                                           // via
                                                           // SysIdRoutineLog.linearVelocity
                    },
                    this));

    /**
     * Subsystem constructor.
     */
    public ElevatorSubsystem() {
        // kZeroed = false;
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
                .closedLoopRampRate(ElevatorConstants.kElevatorRampRate).closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1);
        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator visualization, select Network Tables -> SmartDashboard
        // -> Elevator Sim

        if (RobotBase.isSimulation()) {
            m_limitSwitchLowSim = new DIOSim(m_limitSwitchLow);
            SmartDashboard.putData("Elevator Low Limit Switch", m_limitSwitchLow);

            m_limitSwitchHighSim = new DIOSim(m_limitSwitchHigh);
            SmartDashboard.putData("Elevator Low Limit Switch", m_limitSwitchHigh);
        }
    }

    /**
     * Advance the simulation.
     */
    public void simulationPeriodic() {
        if (m_motorSim.getRelativeEncoderSim().getPosition() <= 0.02) {
            m_limitSwitchLowSim.setValue(true);
        } else {
            m_limitSwitchLowSim.setValue(false);
        }

        if (m_motorSim.getRelativeEncoderSim().getPosition() >= 200.0) {
            m_limitSwitchHighSim.setValue(true);
        } else {
            m_limitSwitchHighSim.setValue(false);
        }
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_motorSim.iterate(
                Elevator
                        .convertDistanceToRotations(
                                Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
                        .per(Second).in(RPM),
                RoboRioSim.getVInVoltage(),
                0.020);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public void reachGoal(double goal) {
        double voltsOut = MathUtil.clamp(
                m_controller.calculate(getHeightMeters(), goal) +
                        m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                m_controller.getSetpoint().velocity),
                -7, 7);
        m_motor.setVoltage(voltsOut);
    }

    /**
     * Runs the SysId routine to tune the Arm
     *
     * @return SysId Routine command
     */
    public Command runSysIdRoutine() {
        return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
                .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
                .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
                .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
                .andThen(Commands.print("DONE"));
    }

    /**
     * Get the height in meters.
     *
     * @return Height in meters
     */
    public double getHeightMeters() {
        return (m_encoder.getPosition() / ElevatorConstants.kElevatorGearing) *
                (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
    }

    /**
     * The velocity of the elevator in meters per second.
     *
     * @return velocity in meters per second
     */
    public double getVelocityMetersPerSecond() {
        return ((m_encoder.getVelocity() / 60) / ElevatorConstants.kElevatorGearing) *
                (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
    }

    /**
     * A trigger for when the height is at an acceptable tolerance.
     *
     * @param height    Height in Meters
     * @param tolerance Tolerance in meters.
     * @return {@link Trigger}
     */
    public Trigger atHeight(double height, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(height,
                getHeightMeters(),
                tolerance));
    }

    /**
     * Set the goal of the elevator
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command setGoal(double goal) {
        return run(() -> reachGoal(goal));
    }

    /**
     * Stop the control loop and motor output.
     */
    public void stop() {
        m_motor.set(0.0);
    }

    /**
     * Update telemetry, including the mechanism visualization.
     */
    public void updateTelemetry() {
        // Update elevator visualization with position
        Constants.kElevatorCenterStage.setLength(getHeightMeters());
        Constants.kElevatorCarriage.setLength(getHeightMeters());
    }

    @Override
    public void periodic() {
    }

    public void zeroElevator() {
        // this.kZeroed = true;
        m_encoder.setPosition(0.0);
    }
}
