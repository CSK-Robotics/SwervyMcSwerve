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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
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

    private SparkFlex m_motor = new SparkFlex(13, MotorType.kBrushless);
    //private SparkFlex m_motor2 = new SparkFlex(14, MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_motor.getEncoder();
    private SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
    // This gearbox represents a gearbox containing 2 Neos
    // Either we have 2 gear boxes(1 for each Neo) or 1 gear box with 2 Neos//
    // private final DCMotor m_armGearbox = DCMotor.getNEO(1).withReduction(25.0);

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(AlgaeConstants.kArmMaxVelocity,
                    AlgaeConstants.kArmMaxAcceleration));

    // private DigitalInput m_limitSwitch = new DigitalInput(0);

    public Algae() {
        m_motor.configure(new SparkFlexConfig().apply(new ClosedLoopConfig()), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        //m_motor2.configure(new SparkFlexConfig().follow(m_motor, true), ResetMode.kResetSafeParameters,
        //        PersistMode.kPersistParameters);
    }

    public void runWheel(double speed) {
        m_motor.setVoltage(speed *12);
    }

    public void stop() {
        m_motor.stopMotor();
    }
}
