package frc.robot.HumanInterface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Commands.Orchestrator;
import frc.robot.Commands.Orchestrator.ReefPosition;
import frc.robot.Subsystems.AlgaeSubsystem.Position;

public class OperatorControls {
    private final XboxController m_controller = new XboxController(1);

    public final EventLoop m_loop = new EventLoop();

    public OperatorControls(Orchestrator orchestrator) {
        m_controller.a(m_loop).debounce(0.2).ifHigh(() -> orchestrator.ScoreCoral(ReefPosition.L1, false));
        m_controller.b(m_loop).debounce(0.2).ifHigh(() -> orchestrator.ScoreCoral(ReefPosition.L2, false));
        m_controller.x(m_loop).debounce(0.2).ifHigh(() -> orchestrator.ScoreCoral(ReefPosition.L3, false));
        m_controller.y(m_loop).debounce(0.2).ifHigh(() -> orchestrator.ScoreCoral(ReefPosition.L4, false));
        m_controller.leftBumper(m_loop).debounce(0.2).ifHigh(() -> orchestrator.IntakeCoral(false));
        m_controller.rightBumper(m_loop).debounce(0.2).ifHigh(() -> orchestrator.ScoreAlgae(Position.BARGE, false));
    }
}
