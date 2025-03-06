package frc.robot.Commands;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
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

    public Command AimCoral(CoralSubsystem.Position position, boolean autoAlign) {
        return m_elevator.setGoal(ElevatorSubsystem.Position.valueOf(position.toString()))
                .alongWith(m_coral.aimCoral(position));
    }

    public Command IntakeCoral(boolean autoAlign) {
        return AimCoral(CoralSubsystem.Position.HPSTATION, autoAlign).andThen(m_coral.intakeCoral());
    }

    public Command ScoreCoral(CoralSubsystem.Position position, boolean autoAlign, BooleanEvent fire) {
        return AimCoral(position, autoAlign)
                .andThen(m_coral.scoreCoral(position, fire));
    }

    public Command AimAlgae(AlgaeSubsystem.Position position, boolean autoAlign) {
        return m_elevator.setGoal(ElevatorSubsystem.Position.valueOf(position.toString()))
                .alongWith(m_algae.aimAlgae(position));
    }

    public Command IntakeAlgae(AlgaeSubsystem.Position level, boolean autoAlign) {
        if (level != AlgaeSubsystem.Position.L2
                && level != AlgaeSubsystem.Position.L3) {
            return null;
        }
        return AimAlgae(level, autoAlign).andThen(m_algae.intakeAlgae(level));
    }

    public Command ScoreAlgae(AlgaeSubsystem.Position bargeOrProcessor, boolean autoAlign, BooleanEvent fire) {
        if (bargeOrProcessor != AlgaeSubsystem.Position.BARGE
                && bargeOrProcessor != AlgaeSubsystem.Position.PROCESSOR) {
            return null;
        }
        return AimAlgae(bargeOrProcessor, autoAlign).andThen(m_algae.scoreAlgae(bargeOrProcessor, fire));
    }

    public Command Climb() {
        return m_climber.setGoal(Position.LIFTED);
    }

    public Command Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        return m_drivetrain.run(() -> m_drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative));
    }
}
