package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber {
    private SparkMax m_climberMotor = new SparkMax(11, MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_climberMotor.getEncoder();
    private SparkClosedLoopController m_controller = m_climberMotor.getClosedLoopController();

    public Climber() {
        m_climberMotor.configure(new SparkMaxConfig().apply(new ClosedLoopConfig()), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climb(double speed) {
        m_controller.setReference(speed, ControlType.kVoltage);
    }

    public void stop() {
        m_climberMotor.stopMotor();
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }
}
