package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ClimberSubsystem.Position;

public class Orchestrator {
    public enum ReefPosition {
        L1,
        L2,
        L3,
        L4
    }

    private final DrivetrainSubsystem m_drivetrain;
    private final ElevatorSubsystem m_elevator;
    private final CoralSubsystem m_coral;
    private final AlgaeSubsystem m_algae;
    private final ClimberSubsystem m_climber;

    public Orchestrator(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, CoralSubsystem coral,
            AlgaeSubsystem algae, ClimberSubsystem climber) {
        m_drivetrain = drivetrain;
        m_elevator = elevator;
        m_coral = coral;
        m_algae = algae;
        m_climber = climber;
    }

    public Command IntakeCoral(boolean autoAlign) {
        return new ParallelCommandGroup(m_elevator.setGoal(ElevatorSubsystem.Position.HPSTATION),
                m_coral.setGoal(CoralSubsystem.Position.HPSTATION));
    }

    public Command ScoreCoral(ReefPosition position, boolean autoAlign) {
        return new ParallelCommandGroup(m_elevator.setGoal(ElevatorSubsystem.Position.valueOf(position.toString())),
                m_coral.setGoal(CoralSubsystem.Position.valueOf(position.toString())));
    }

    public Command IntakeAlgae(AlgaeSubsystem.Position level, boolean autoAlign) {
        if (level != AlgaeSubsystem.Position.L2
                && level != AlgaeSubsystem.Position.L3) {
            return null;
        }
        return new SequentialCommandGroup(
                m_elevator.setGoal(ElevatorSubsystem.Position.valueOf(level.toString())),
                m_algae.setGoal(level));
    }

    public Command ScoreAlgae(AlgaeSubsystem.Position bargeOrProcessor, boolean autoAlign) {
        if (bargeOrProcessor != AlgaeSubsystem.Position.BARGE
                && bargeOrProcessor != AlgaeSubsystem.Position.PROCESSOR) {
            return null;
        }
        return new SequentialCommandGroup(
                m_elevator.setGoal(ElevatorSubsystem.Position.valueOf(bargeOrProcessor.toString())),
                m_algae.setGoal(bargeOrProcessor));
    }

    public Command Climb() {
        return m_climber.setGoal(Position.LIFTED);
    }

    public Command Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        return m_drivetrain.run(() -> m_drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative));
    }
}
