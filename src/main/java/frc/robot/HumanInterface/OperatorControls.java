package frc.robot.HumanInterface;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Orchestrator;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.AlgaeSubsystem;

public class OperatorControls {
    private final XboxController m_controller = new XboxController(1);

    public final EventLoop m_loop = new EventLoop();
    public final Map<String, Trigger> m_triggers = new HashMap<String, Trigger>();

    public OperatorControls(Orchestrator orchestrator) {
        m_triggers.put("A: L1 Coral", new Trigger(m_controller.a(m_loop).debounce(0.2))
                .onTrue(orchestrator.ScoreCoral(CoralSubsystem.Position.L1, false, m_controller.rightBumper(m_loop))));
        m_triggers.put("B: L2 Coral", new Trigger(m_controller.b(m_loop).debounce(0.2))
                .onTrue(orchestrator.ScoreCoral(CoralSubsystem.Position.L2, false, m_controller.rightBumper(m_loop))));
        m_triggers.put("X: L3 Coral", new Trigger(m_controller.x(m_loop).debounce(0.2))
                .onTrue(orchestrator.ScoreCoral(CoralSubsystem.Position.L3, false, m_controller.rightBumper(m_loop))));
        m_triggers.put("Y: L4 Coral", new Trigger(m_controller.y(m_loop).debounce(0.2))
                .onTrue(orchestrator.ScoreCoral(CoralSubsystem.Position.L4, false, m_controller.rightBumper(m_loop))));
        m_triggers.put("LB: Intake Coral",
                new Trigger(m_controller.leftBumper(m_loop).debounce(0.2)).onTrue(orchestrator.IntakeCoral(false)));

        m_triggers.put("UP: Net Algae", new Trigger(m_controller.povUp(m_loop).debounce(0.2)).onTrue(orchestrator
                .ScoreAlgae(AlgaeSubsystem.Position.BARGE, false, m_controller.rightTrigger(0.25, m_loop))));
        m_triggers.put("DOWN: Processor Algae",
                new Trigger(m_controller.povDown(m_loop).debounce(0.2)).onTrue(orchestrator.ScoreAlgae(
                        AlgaeSubsystem.Position.PROCESSOR, false, m_controller.rightTrigger(0.25, m_loop))));
        m_triggers.put("LEFT: Intake L3 Algae", new Trigger(m_controller.povLeft(m_loop).debounce(0.2))
                .onTrue(orchestrator.IntakeAlgae(AlgaeSubsystem.Position.L2, false)));
        m_triggers.put("RIGHT: Intake L2 Algae", new Trigger(m_controller.povRight(m_loop).debounce(0.2))
                .onTrue(orchestrator.IntakeAlgae(AlgaeSubsystem.Position.L2, false)));
    }
}
