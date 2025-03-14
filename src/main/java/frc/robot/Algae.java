package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {
    // Zeroed state
    private boolean kZeroed;

    private SparkMax m_motor = new SparkMax(11, MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_motor.getEncoder();
    private SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
    // This gearbox represents a gearbox containing 2 Neos
    // Either we have 2 gear boxes(1 for each Neo) or 1 gear box with 2 Neos//
    //private final DCMotor m_armGearbox = DCMotor.getNEO(1).withReduction(25.0);

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(AlgaeConstants.kArmMaxVelocity,
                    AlgaeConstants.kArmMaxAcceleration));

    private DigitalInput m_limitSwitch = new DigitalInput(0);

    public Trigger atMin = new Trigger(m_limitSwitch::get).onTrue(this.run(() -> {
        stop();
        m_encoder.setPosition(0.0);
        kZeroed = true;
    }));

    public Algae() {
        m_motor.configure(new SparkMaxConfig().apply(new ClosedLoopConfig()), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * Zero the elevator
     *
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public void zeroArm() {
        // return runEnd(() -> {
        if (kZeroed) {
            reachGoal(0.0);
        } else {
            // double feedforward =
            // m_feedforward.calculateWithVelocities(m_encoder.getVelocity(),
            // ElevatorConstants.kZeroingSpeed);
            m_controller.setReference(AlgaeConstants.kZeroingSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            // feedforward);
        }
    }// , this::stop).until(atMin);

    /**
     * Run control loop to reach and maintain goal.
     *
     * @param goal the position to maintain
     */
    public void reachGoal(double goal) {
        State setpoint = m_profile.calculate(TimedRobot.kDefaultPeriod,
                new State(m_encoder.getPosition(), m_encoder.getVelocity()),
                new State(goal, 0.0));

        // double feedforward =
        // m_feedforward.calculateWithVelocities(m_encoder.getVelocity(),
        // setpoint.velocity);
        m_controller.setReference(setpoint.velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void climb(double speed) {
        m_controller.setReference(speed, ControlType.kVoltage);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }
}
