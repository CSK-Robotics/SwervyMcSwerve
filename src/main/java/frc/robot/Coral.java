package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
    // Zeroed state
    private boolean kZeroed;

    private SparkMax m_motor = new SparkMax(11, MotorType.kBrushless);
    private SparkFlex m_wheelMotor = new SparkFlex(12, MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_motor.getEncoder();
    private SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
    // This gearbox represents a gearbox containing 2 Neos
    // Either we have 2 gear boxes(1 for each Neo) or 1 gear box with 2 Neos//
    //private final DCMotor m_armGearbox = DCMotor.getNEO(1).withReduction(25.0);

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(CoralConstants.kArmMaxVelocity,
                    CoralConstants.kArmMaxAcceleration));

    private DigitalInput m_limitSwitch = new DigitalInput(1);

    public Trigger atMin = new Trigger(m_limitSwitch::get).onFalse(this.runOnce(() -> {
        stop();
        System.out.println("I aM ToUchIng tHe LimIt SwiTcH");
        m_encoder.setPosition(0.0);
        kZeroed = true;
    }));

    public Coral() {
        m_motor.configure(new SparkMaxConfig().apply(new ClosedLoopConfig()).apply(new EncoderConfig().positionConversionFactor(360 / (Math.PI * 2))), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_wheelMotor.configure(new SparkFlexConfig().apply(new ClosedLoopConfig()), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
            m_controller.setReference(CoralConstants.kZeroingSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
        m_controller.setReference(goal, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Coral Zeroed", !m_limitSwitch.get());
        SmartDashboard.putNumber("Coral Angle", getAngle());
    }

    public void runWheel(double speed) {
        m_wheelMotor.setVoltage(speed *6);
    }
}
